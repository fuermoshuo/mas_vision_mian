#!/usr/bin/env python3
"""
mas_vision plotter 接收端

接收 rm_utils::Plotter 通过 UDP 发出的 JSON 数据，实时绘图 + 录制。

用法
----
    # 实时绘图（默认视图：小陀螺诊断）
    python3 tools/plot_monitor.py

    # 指定预设视图
    python3 tools/plot_monitor.py --view jitter
    python3 tools/plot_monitor.py --view drift

    # 自定义曲线（逗号分隔，支持嵌套 key)
    python3 tools/plot_monitor.py --keys w,r,send_packet.target_yaw

    # 只录制不绘图（无第三方依赖）
    python3 tools/plot_monitor.py --no-gui

    # 列出当前实际收到的所有字段名
    python3 tools/plot_monitor.py --list

依赖
----
    绘图模式:pip3 install pyqtgraph PyQt5
    --no-gui / --list 模式：仅需标准库

说明
----
* C++ 侧默认发往 127.0.0.1:9870(见 rm_utils/plotter/plotter.hpp),
  所以本脚本默认要跑在 NUC 本机上。
  要在别的机器上看，改 Plotter 构造参数的 host,或用 --bind 0.0.0.0 配合转发。
* armor_track 和 armor_shoot 发的是两个独立的 UDP 包，字段集不同。
  脚本按"字段集指纹"自动分流，分别录成不同的 CSV。
* 录制默认开启且始终进行。调车真正的分析是事后对比两次跑的数据，
  不是盯着实时曲线看。
"""

import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import sys
import threading
import time
from collections import deque

# ---------------------------------------------------------------- 预设视图

PRESET_VIEWS = {
    # 小陀螺诊断：确认模式抖动、EKF 双重更新
    "spin": [
        ("w",                      "EKF 角速度估计 (rad/s)"),
        ("update_count",           "EKF 更新次数（斜率应≈帧率）"),
        ("aim_point.x",            "瞄准点 X (m)"),
        ("aim_point.y",            "瞄准点 Y (m)"),
    ],
    # 指令噪声诊断：确认刺啦声来源
    "jitter": [
        ("send_packet.target_yaw",   "发给电控的 yaw (deg)"),
        ("gimbal.yaw",               "云台当前 yaw (deg)"),
        ("send_packet.target_pitch", "发给电控的 pitch (deg)"),
        ("send_packet.found",        "found 标志"),
    ],
    # 上飘诊断
    "drift": [
        ("z",                      "旋转中心 Z (m)"),
        ("vz",                     "Z 方向速度 (m/s)"),
        ("x",                      "旋转中心 X (m)"),
        ("vx",                     "X 方向速度 (m/s)"),
    ],
    # EKF 收敛质量
    "converge": [
        ("r",                      "旋转半径 (m)"),
        ("l",                      "大板半径增量 (m)"),
        ("h",                      "高度差 (m)"),
        ("w",                      "角速度 (rad/s)"),
    ],
}

DEFAULT_VIEW = "spin"


class _Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, value):
        for stream in self.streams:
            stream.write(value)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()


def default_run_dir():
    root = Path(__file__).resolve().parents[1]
    base = root / "plot_logs"
    stamp = time.strftime("run_%Y%m%d_%H%M%S")
    run_dir = base / stamp
    suffix = 1
    while run_dir.exists():
        run_dir = base / f"{stamp}_{suffix:02d}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def find_base_log():
    log_dir = Path(__file__).resolve().parents[1] / "build" / "logs"
    logs = list(log_dir.glob("mas_vision_*.log"))
    return max(logs, key=lambda path: path.stat().st_mtime) if logs else None

# ---------------------------------------------------------------- 工具


def flatten(obj, prefix=""):
    """把嵌套 JSON 拍平成 {'a.b.c': value}，只保留数值和布尔。"""
    out = {}
    for k, v in obj.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        elif isinstance(v, bool):
            out[key] = 1.0 if v else 0.0
        elif isinstance(v, (int, float)):
            out[key] = float(v)
        else:
            # 字符串字段（state / target_name / armor_type）单独留着，CSV 要用
            out[key] = v
    return out


class Recorder:
    """按字段集指纹自动分流，每个流一个 CSV。"""

    def __init__(self, outdir):
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.streams = {}  # fingerprint -> (file, writer, fieldnames)
        self.lock = threading.Lock()

    def write(self, row):
        fp = tuple(sorted(row.keys()))
        with self.lock:
            if fp not in self.streams:
                idx = len(self.streams)
                path = os.path.join(self.outdir, f"plot_{self.stamp}_s{idx}.csv")
                f = open(path, "w", newline="", encoding="utf-8")
                fields = list(fp)
                w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
                w.writeheader()
                self.streams[fp] = (f, w, fields)
                print(f"[rec] 新数据流 -> {path}")
                print(f"[rec]   字段: {', '.join(fp)}")
            f, w, _ = self.streams[fp]
            w.writerow(row)

    def close(self):
        with self.lock:
            for f, _, _ in self.streams.values():
                f.close()
        if self.streams:
            print(f"[rec] 已保存 {len(self.streams)} 个 CSV 到 {self.outdir}/")


class Receiver(threading.Thread):
    """UDP 接收线程。收到就拍平、盖时间戳、录制、塞进环形缓冲。"""

    daemon = True

    def __init__(self, bind, port, recorder, buffers, buflen, seen_keys):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 加大接收缓冲，避免 GUI 卡顿时丢包
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.5)
        self.recorder = recorder
        self.buffers = buffers
        self.buflen = buflen
        self.seen_keys = seen_keys
        self.running = True
        self.count = 0
        self.bad = 0
        self.t0 = time.monotonic()

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                obj = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.bad += 1
                continue

            if not isinstance(obj, dict):
                self.bad += 1
                continue

            row = flatten(obj)
            t = time.monotonic() - self.t0
            self.count += 1

            self.seen_keys.update(row.keys())

            rec_row = dict(row)
            rec_row["t_host"] = round(t, 6)
            self.recorder.write(rec_row)

            for k, v in row.items():
                if not isinstance(v, float):
                    continue
                buf = self.buffers.get(k)
                if buf is None:
                    buf = (deque(maxlen=self.buflen), deque(maxlen=self.buflen))
                    self.buffers[k] = buf
                buf[0].append(t)
                buf[1].append(v)

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 无 GUI 模式


def run_headless(rx, seen_keys, list_only):
    last = 0
    last_t = time.monotonic()
    def _stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    try:
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            rate = (rx.count - last) / (now - last_t)
            last, last_t = rx.count, now

            if list_only:
                if seen_keys:
                    print(f"\n收到 {rx.count} 包，字段列表：")
                    for k in sorted(seen_keys):
                        print(f"  {k}")
                    return
                print("等待数据... (确认 auto_aim.yaml 里 plotter_enable / debug 为 true)")
            else:
                print(f"[rx] 共 {rx.count} 包  {rate:5.1f} pkt/s  解析失败 {rx.bad}")
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------- GUI 模式


def run_gui(rx, curves, window_sec, refresh_hz):
    try:
        import numpy as np
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except ImportError:
        print("缺少依赖。请先安装：\n    pip3 install pyqtgraph PyQt5\n"
              "或者用 --no-gui 只录制。", file=sys.stderr)
        return 1

    # 低 CPU 配置：关抗锯齿，白底（比黑底省电，且投影仪可见）
    pg.setConfigOptions(antialias=False, background="w", foreground="k")

    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(title="mas_vision plotter")
    win.resize(1100, 760)

    plots = []
    for i, (key, label) in enumerate(curves):
        p = win.addPlot(row=i, col=0)
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setLabel("left", label)
        p.setMouseEnabled(x=False, y=True)
        if i == len(curves) - 1:
            p.setLabel("bottom", "时间 (s)")
        curve = p.plot(pen=pg.mkPen(pg.intColor(i, len(curves)), width=2))
        txt = pg.TextItem(anchor=(0, 1), color="k")
        p.addItem(txt)
        plots.append((key, p, curve, txt))

    win.show()

    status = {"last_count": 0, "last_t": time.monotonic()}

    def update():
        now = time.monotonic()
        dt = now - status["last_t"]
        if dt >= 1.0:
            rate = (rx.count - status["last_count"]) / dt
            status["last_count"], status["last_t"] = rx.count, now
            win.setWindowTitle(
                f"mas_vision plotter — {rate:.0f} pkt/s — 共 {rx.count} 包"
            )

        for key, p, curve, txt in plots:
            buf = rx.buffers.get(key)
            if not buf or len(buf[0]) < 2:
                continue
            # deque -> numpy，一次性转换比逐点 append 快得多
            xs = np.fromiter(buf[0], dtype=np.float64, count=len(buf[0]))
            ys = np.fromiter(buf[1], dtype=np.float64, count=len(buf[1]))

            t_end = xs[-1]
            mask = xs >= (t_end - window_sec)
            xs, ys = xs[mask], ys[mask]
            if xs.size < 2:
                continue

            curve.setData(xs, ys)
            p.setXRange(t_end - window_sec, t_end, padding=0)

            # 右上角显示当前值和窗口内峰峰值——判断"抖不抖"最直接的指标
            pp = float(ys.max() - ys.min())
            txt.setText(f"now={ys[-1]:+.4f}   峰峰={pp:.4f}")
            txt.setPos(t_end - window_sec, float(ys.max()))

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    # 刷新率与数据率解耦：数据 89Hz 进缓冲，画面固定低频重绘。
    # 这是本脚本 CPU 占用低的主要原因。
    timer.start(int(1000 / max(refresh_hz, 1.0)))

    # Qt 事件循环默认不处理 SIGINT；Ctrl+C 会在 update() 里反复抛 KeyboardInterrupt 却退不掉。
    # 空定时器把控制权交回 Python，SIGINT 时直接 quit。
    def _quit(*_):
        app.quit()

    signal.signal(signal.SIGINT, _quit)
    try:
        signal.signal(signal.SIGTERM, _quit)
    except (AttributeError, ValueError):
        pass

    wake = QtCore.QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(200)

    try:
        if hasattr(app, "exec_"):
            app.exec_()
        else:
            app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        timer.stop()
        wake.stop()
        win.close()
    return 0


# ---------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser(
        description="mas_vision plotter 接收端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    ap.add_argument("--port", type=int, default=9870, help="监听端口（默认 9870）")
    ap.add_argument("--view", choices=sorted(PRESET_VIEWS), default=DEFAULT_VIEW,
                    help=f"预设视图（默认 {DEFAULT_VIEW}）")
    ap.add_argument("--keys", default=None,
                    help="自定义曲线，逗号分隔，覆盖 --view")
    ap.add_argument("--window", type=float, default=10.0, help="横轴时间窗口秒数")
    ap.add_argument("--refresh", type=float, default=25.0, help="画面刷新率 Hz")
    ap.add_argument("--buflen", type=int, default=20000, help="每条曲线的环形缓冲长度")
    ap.add_argument("--outdir", default=None, help="CSV 输出目录；省略时自动创建 plot_logs/run_时间")
    ap.add_argument("--no-gui", action="store_true", help="只录制，不绘图")
    ap.add_argument("--list", action="store_true", help="列出收到的所有字段名后退出")
    args = ap.parse_args()

    if args.keys:
        curves = [(k.strip(), k.strip()) for k in args.keys.split(",") if k.strip()]
    else:
        curves = PRESET_VIEWS[args.view]

    project_root = Path(__file__).resolve().parents[1]
    plot_logs_dir = project_root / "plot_logs"
    outdir = Path(args.outdir).expanduser().resolve() if args.outdir else default_run_dir()
    outdir.mkdir(parents=True, exist_ok=True)
    plot_logs_dir.mkdir(parents=True, exist_ok=True)
    original_stdout, original_stderr = sys.stdout, sys.stderr
    monitor_log = open(outdir / "plot_monitor.log", "a", encoding="utf-8", buffering=1)
    sys.stdout = _Tee(original_stdout, monitor_log)
    sys.stderr = _Tee(original_stderr, monitor_log)
    recorder = Recorder(str(outdir))
    buffers = {}
    seen_keys = set()

    try:
        rx = Receiver(args.bind, args.port, recorder, buffers, args.buflen, seen_keys)
    except OSError as e:
        print(f"绑定 {args.bind}:{args.port} 失败: {e}", file=sys.stderr)
        sys.stdout, sys.stderr = original_stdout, original_stderr
        monitor_log.close()
        return 1

    pid_file = plot_logs_dir / "plot_monitor.pid"
    (plot_logs_dir / "latest_run.txt").write_text(str(outdir) + "\n", encoding="utf-8")
    pid_file.write_text(str(os.getpid()) + "\n", encoding="ascii")
    rx.buffers = buffers
    rx.start()

    print(f"监听 {args.bind}:{args.port}")
    print(f"录制目录 {outdir}/")
    if not args.no_gui and not args.list:
        print(f"视图 [{args.view}]: " + ", ".join(k for k, _ in curves))
    print("Ctrl+C 退出\n")

    try:
        if args.list or args.no_gui:
            run_headless(rx, seen_keys, args.list)
        else:
            run_gui(rx, curves, args.window, args.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()
        rx.join()
        recorder.close()
        print(f"\n共接收 {rx.count} 包，解析失败 {rx.bad} 包")
        base_log = find_base_log()
        if base_log and base_log.exists():
            shutil.copy2(base_log, outdir / "base.log")
            print(f"程序日志 -> {outdir / 'base.log'}")
        else:
            print("未找到 build/logs/mas_vision_*.log，未复制程序日志")
        if pid_file.exists() and pid_file.read_text(encoding="ascii").strip() == str(os.getpid()):
            pid_file.unlink()
        sys.stdout, sys.stderr = original_stdout, original_stderr
        monitor_log.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
