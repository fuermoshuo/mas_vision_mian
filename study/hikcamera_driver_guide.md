# 海康相机驱动 (HikCamera) 深度详解

> 从零开始，循序渐进理解海康相机驱动的三线程架构与四个源文件的协作关系。

---

## 目录

1. [快速概览：这个驱动是干什么的？](#1-快速概览)
2. [架构总览：三线程模型](#2-架构总览)
3. [hikcamera.hpp — 蓝图：类结构与设计意图](#3-hikcamerahpp--蓝图)
4. [hikcamera.cpp — 生命周期：从构造到销毁](#4-hikcameracpp--生命周期)
5. [hikcamera_capture.cpp — 采集管线：帧从何来、去往何处](#5-hikcamera_capturecpp--采集管线)
6. [hikcamera_reconnect.cpp — 守护自愈：断线检测与自动重连](#6-hikcamera_reconnectcpp--守护自愈)
7. [三者协同全景图](#7-三者协同全景图)

---

## 1. 快速概览

这个驱动封装了海康工业相机的完整操作，提供：

| 能力 | 说明 |
|------|------|
| **打开相机** | 枚举设备、创建句柄、配置曝光/增益、启动取流 |
| **采集图像** | 独立线程持续抓帧，Bayer→BGR 解码，推入帧队列 |
| **获取图像** | 上层调用 `getImage()` 从队列取帧（阻塞等待） |
| **断线自愈** | 守护线程监测心跳，掉线后自动重连（退避策略） |
| **安全退出** | 线程安全停止、资源释放、SDK 反初始化 |

---

## 2. 架构总览

驱动内部有**三个并发执行的上下文**：

```
┌─────────────────────────────────────────────────────┐
│  主线程 (用户调用线程)                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │openCamera│  │ getImage │  │closeCamera│          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘          │
│       │              │              │                │
│       ▼              ▼              ▼                │
│  ┌───────────────────────────────────────┐          │
│  │         camera_mutex_ (定时锁)         │          │
│  │         保护 handle / SDK 调用          │          │
│  └───────────────────────────────────────┘          │
│       ▲              ▲              ▲                │
│       │              │              │                │
│  ┌────┴─────┐  ┌────┴─────┐  ┌────┴─────┐          │
│  │captureLoop│  │daemonLoop│  │ TryConnect│          │
│  │ (采帧线程)│  │ (守护线程)│  │  (重连)   │          │
│  └──────────┘  └──────────┘  └──────────┘          │
│                                                      │
│  ┌──────────────────────────────────────┐           │
│  │  frame_queue_ (queue_mutex_ 保护)     │           │
│  │  ← pushFrame() 入队                  │           │
│  │  ← getImage()  出队                  │           │
│  └──────────────────────────────────────┘           │
└─────────────────────────────────────────────────────┘
```

**核心思想：生产者-消费者 + 看门狗**

- **captureLoop** = 生产者：从相机抓帧 → 转换 → 推入队列
- **getImage**  = 消费者：从队列取帧 → 返回给上层
- **daemonLoop** = 看门狗：监测心跳 → 触发重连

---

## 3. hikcamera.hpp — 蓝图

头文件是理解整个驱动的**地图**。先看它再读 `.cpp`，思路不会乱。

### 3.1 类继承关系

```cpp
class HikCamera : public Base_Camera
```

`Base_Camera` 是项目中所有相机驱动的抽象基类，定义了三个纯虚函数：

| 虚函数 | 作用 |
|--------|------|
| `openCamera()` | 打开相机 |
| `closeCamera()` | 关闭相机 |
| `getImage()` | 获取一帧图像 |

`HikCamera` 实现这三个接口，上层代码无需关心底层是海康还是其他品牌。

### 3.2 禁止拷贝

```cpp
HikCamera(const HikCamera&) = delete;
HikCamera& operator=(const HikCamera&) = delete;
```

相机句柄 (`void *handle`) 是海康 SDK 分配的**独占资源**，拷贝会导致双重释放。`= delete` 在编译期就阻止了这种危险操作。

### 3.3 公有接口一览

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `HikCamera(config_path)` | — | 构造函数，从 YAML 加载曝光/增益/序列号 |
| `~HikCamera()` | — | 析构：停线程 → 关相机 → SDK反初始化 |
| `openCamera()` | `bool` | 枚举设备、创建句柄、配置参数、开始取流；**首次调用自动启动内部线程** |
| `closeCamera()` | `void` | 停止取流、关闭设备、销毁句柄 |
| `getImage()` | `CameraFrame` | 从帧队列取一帧，阻塞最多 `kQueueWaitMs` 毫秒 |
| `isConnectedStatus()` | `bool` | 无锁读取当前连接状态 |

### 3.4 私有成员变量详解

#### 核心资源

| 变量 | 类型 | 含义 |
|------|------|------|
| `handle` | `void*` | 海康 SDK 相机句柄，所有 SDK 调用的"身份证"。受 `camera_mutex_` 保护 |
| `isConnected` | `atomic<bool>` | 连接状态，无锁读取，`isConnectedStatus()` 直接返回它 |
| `serialNumber` | `char[64]` | 设备的硬件序列号（从 SDK 读取） |
| `target_serial_` | `string` | YAML 中指定的目标序列号，空串 = 不限定（取第一个枚举到的设备） |

#### 状态标记

| 变量 | 类型 | 含义 |
|------|------|------|
| `sdk_initialized_` | `bool` | SDK 是否已调用 `MV_CC_Initialize()` |
| `wedge_detected_` | `bool` | 采集线程是否疑似**卡死**（长时间占锁不释放） |
| `was_connected_` | `bool` | 上一轮 daemon 轮询时的连接状态（用于检测**状态变化沿**） |
| `was_stalled_` | `bool` | 上一轮 daemon 轮询时的心跳停滞状态 |
| `first_frame_pending_` | `atomic<bool>` | 重连后等待首帧（标记重连恢复延迟统计是否待打印） |

#### 计数器

| 变量 | 类型 | 含义 |
|------|------|------|
| `consecutive_sdk_errors_` | `int` | 连续 SDK 错误计数（累计到阈值触发断线判定） |
| `wedge_consecutive_misses_` | `int` | 连续抢锁失败次数（去抖：攒够 N 次才认为可能卡死） |
| `reconnect_backoff_ms_` | `int` | 当前退避间隔（失败翻倍，成功重置） |
| `pool_index` | `int` | 小池化输出 Mat 的轮转索引（0~4 循环） |

#### 时间戳（断线恢复延迟诊断用）

| 变量 | 类型 | 含义 |
|------|------|------|
| `t_disconnect_signal_ms_` | `atomic<int64_t>` | 断线信号触发时刻（毫秒 epoch） |
| `t_grab_started_ms_` | `atomic<int64_t>` | `StartGrabbing` 成功的时刻 |
| `last_frame_tick_ms_` | `atomic<int64_t>` | 采集线程心跳：最近一次成功抓帧的时间 |
| `first_frame_pending_` | `atomic<bool>` | 重连后首个帧到达时打印恢复延迟 |

### 3.5 所有常量详解

这是理解时间参数的关键。**数字不是魔法，每个都有明确的工程理由：**

```cpp
// ─── 重连退避 ───
static constexpr int kInitialBackoffMs = 200;   // 首次重连间隔 200ms
static constexpr int kMaxBackoffMs     = 500;   // 最大退避上限 500ms（失败了也要持续尝试）

// ─── 日志节流 ───
static constexpr int kFailLogThrottleMs = 2000; // 重连失败日志每 2 秒最多打印一条（防刷屏）

// ─── 采集管线 ───
static constexpr int kPoolCount               = 5;   // 小池化 Mat 缓冲区数量（减少内存分配）
static constexpr int kMaxConsecutiveSdkErrors = 5;   // 连续 >5 次非 NODATA 错误 → 判定为断线

// ─── 守护线程 ───
static constexpr int kDaemonPollMs      = 100;   // 每 100ms 轮询一次
static constexpr int kWatchdogTimeoutMs = 1000;  // 超过 1000ms 无新帧 → 心跳超时
static constexpr int kWedgeEscalateMs   = 10000; // 锁卡死超过 10 秒 → 升级为 ERROR 日志
static constexpr int kWedgeLogDebounceMisses = 3; // 连续 3 次抢锁失败 → 判定可能卡死
static constexpr int kWedgeLogThrottleMs     = 2000; // 卡死日志节流 2 秒

// ─── 队列 ───
static constexpr size_t kQueueCapacity = 2;   // 帧队列容量 2 帧（只保留最新）
static constexpr int    kQueueWaitMs   = 100; // getImage 最多等 100ms

// ─── 超时 ───
static constexpr int kCloseTeardownWarnMs     = 150; // 关闭超过 150ms 打印警告
static constexpr int kGetImageBufferTimeoutMs = 20;  // 单次 SDK 取帧内部超时
static constexpr int kCameraLockTimeoutMs     = 200; // 慢路径（open/close）等待锁的上限
static constexpr int kCameraFastLockTimeoutMs = 20;  // 快路径（capture/daemon）等待锁的上限
```

> **为什么有两种锁超时？** `openCamera`/`closeCamera` 是**偶尔发生**的操作，可以等 200ms。但 `captureLoop`/`daemonLoop` 在**100ms 级的高频循环**中，20ms 抢不到就跳过本次（下轮再试），避免阻塞。

### 3.6 线程与同步原语

| 成员 | 类型 | 角色 |
|------|------|------|
| `capture_thread_` | `std::thread` | 采集线程 |
| `daemon_thread_` | `std::thread` | 守护线程 |
| `camera_mutex_` | `std::timed_mutex` | **核心锁**：保护 `handle` 及所有 SDK 调用。使用定时锁是为了防死锁——抢不到就放弃 |
| `queue_mutex_` | `std::mutex` | 保护帧队列 `frame_queue_` |
| `queue_cv_` | `std::condition_variable` | 条件变量：`getImage()` 在队列为空时阻塞于此，`pushFrame()` 来唤醒 |
| `running_` | `atomic<bool>` | 线程启停标志：`true` = 运行中，`false` = 通知退出 |

### 3.7 帧队列

```cpp
std::queue<CameraFrame> frame_queue_;
```

- **容量**：`kQueueCapacity = 2`
- **溢出策略**：丢弃最旧帧（`pop()` 掉队首），推入新帧，保证取到的永远是最新图像
- **同步**：`queue_mutex_` + `queue_cv_`

---

## 4. hikcamera.cpp — 生命周期

这个文件负责**从创建到销毁的完整生命周期**。

### 4.1 构造函数 — `HikCamera(config_path)`

```
步骤1: 初始化列表 → handle=nullptr, isConnected=false, exposure=10000, gain=0
步骤2: 从 YAML 配置文件读取 exposuretime / gain / serial（可选）
步骤3: 异常安全：YAML 解析失败 → 打印错误，使用默认值
```

**参数说明：**

| 参数 | 类型 | 含义 |
|------|------|------|
| `config_path` | `const std::string &` | YAML 配置文件路径，默认 `"config/hikcamera.yaml"` |

> **关键设计：构造函数不打开相机。** 只做配置加载，实际连接延迟到 `openCamera()` 调用时。

### 4.2 析构函数 — `~HikCamera()`

```
步骤1: stopThreads() → 通知线程退出 + join 等待
步骤2: closeCamera() → 停止取流、关闭设备、销毁句柄
步骤3: 若 SDK 已初始化 → MV_CC_Finalize()
```

> **顺序很重要：** 必须先停线程再关相机——否则线程可能在 `closeCamera()` 销毁句柄后继续使用 `handle`，导致崩溃。

### 4.3 `openCamera()` — 打开相机（核心函数）

这是整个驱动最复杂的函数，使用 **goto 错误处理模式**（C 风格资源清理）：

```
获取 camera_mutex_（200ms 超时）
  ├─ 失败 → 返回 false（可能 capture 线程卡死在 SDK 调用中）
  └─ 成功 ↓

已连接且句柄有效？
  ├─ 是 → 返回 true（幂等）
  └─ 否 ↓

SDK 未初始化？
  ├─ MV_CC_Initialize() → 失败则返回 false
  └─ ↓

MV_CC_EnumDevices(USB) → 枚举设备
  ├─ 无设备 → goto clean_init
  └─ 有设备 → 打印设备信息，选第一个

MV_CC_CreateHandle(&handle, pDeviceInfo) → 创建句柄
  ├─ 失败 → goto clean_init
  └─ ↓

MV_CC_OpenDevice(handle) → 打开设备
  ├─ 失败 → goto clean_handle
  └─ ↓

注册异常回调 MV_CC_RegisterExceptionCallBack()
  ├─ 失败 → 仅 WARN（退化为纯心跳检测）
  └─ ↓

设置曝光时间（检查范围 [fMin, fMax]）
设置增益（同上）
设置触发模式 = OFF（连续采集模式）
设置缓存节点数 = 1（只保留最新帧）
设置抓取策略 = LatestImagesOnly
MV_CC_StartGrabbing(handle) → 开始取流
  ├─ 失败 → goto clean_device
  └─ ↓

更新状态：
  isConnected = true
  记录 t_grab_started_ms_（取流开始时间）
  记录 last_frame_tick_ms_（初始化心跳）
  first_frame_pending_ = true（等待首帧到达打印恢复延迟）

startThreads() → 启动采集/守护线程
返回 true

clean_device:  MV_CC_CloseDevice(handle)
clean_handle:  MV_CC_DestroyHandle(handle)
clean_init:    handle=NULL, isConnected=false
               startThreads() → 即使失败也启动线程，让 daemon 自动重试
               返回 false
```

**参数：无。** `openCamera()` 完成后的副作用：
- 相机开始连续采集
- 两个内部线程启动（幂等）

### 4.4 `closeCamera()` — 关闭相机

```
获取 camera_mutex_（200ms 超时）
  ├─ 失败 → 返回（可能正在被占用）
  └─ ↓

if (handle != NULL):
    if (isConnected):
        MV_CC_StopGrabbing(handle)   // 停止取流
        MV_CC_CloseDevice(handle)    // 关闭设备
        isConnected = false
    MV_CC_DestroyHandle(handle)      // 销毁句柄
    handle = NULL
```

### 4.5 `startThreads()` / `stopThreads()` — 线程生命周期

```cpp
void startThreads() {
    if (running_.exchange(true)) return;  // 幂等：已运行则跳过
    capture_thread_ = std::thread(&HikCamera::captureLoop, this);
    daemon_thread_  = std::thread(&HikCamera::daemonLoop, this);
}

void stopThreads() {
    if (!running_.exchange(false)) return; // 幂等
    queue_cv_.notify_all();                // 唤醒阻塞在 getImage() 的消费者
    if (capture_thread_.joinable()) capture_thread_.join();
    if (daemon_thread_.joinable())  daemon_thread_.join();
}
```

> `running_.exchange(val)` 是原子操作：设置新值并返回旧值。利用这个特性实现**幂等**检查——如果旧值已经是目标值，说明状态没变，直接返回。

### 4.6 `markDisconnected()` — 记录断线时刻

```cpp
void markDisconnected(now_steady) {
    if (isConnected.exchange(false)) {  // 仅在"从连接→断开"的瞬间记录时间戳
        t_disconnect_signal_ms_.store(now_ms);
    }
}
```

- `exchange(false)` 将 `isConnected` 设为 false 并返回旧值
- **只在首次断开时记录时间戳**，重复调用不覆盖

### 4.7 `ExceptionCallBack()` — SDK 异常回调

海康 SDK 在检测到设备物理断开时会**异步回调**这个函数：

```cpp
static void ExceptionCallBack(nMsgType, pUser) {
    HikCamera *self = static_cast<HikCamera*>(pUser); // pUser 即 this 指针
    if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT) {
        self->markDisconnected(now);  // 标记断线，daemonLoop 下轮轮询时触发重连
    }
}
```

| 参数 | 类型 | 含义 |
|------|------|------|
| `nMsgType` | `unsigned int` | SDK 异常类型，关注 `MV_EXCEPTION_DEV_DISCONNECT` |
| `pUser` | `void*` | 注册时传入的 `this` 指针，用于回调到成员函数 |

> **为什么是静态函数？** SDK 的 C 接口需要函数指针，C++ 成员函数指针不兼容。用 `static` + `pUser` 绕开。

### 4.8 `PrintDeviceInfo()` — 设备信息打印

根据 `nTLayerType`（传输层类型：GigE/USB/GenTL 等）打印对应协议的信息：

| 传输层 | 打印信息 |
|--------|---------|
| `MV_GIGE_DEVICE` | IP 地址 + 用户自定义名称 |
| `MV_USB_DEVICE` | 用户名称 + 序列号 + 设备编号 |
| `MV_GENTL_*` (4种) | 用户名称 + 序列号 + 型号 |

---

## 5. hikcamera_capture.cpp — 采集管线

这个文件实现了**从相机硬件到用户代码**的完整帧数据流。

### 5.1 数据流全景

```
相机硬件 ──SDK──▶ MV_CC_GetImageBuffer() ──▶ processFrame() ──▶ pushFrame() ──▶ frame_queue_ ──▶ getImage() ──▶ 上层代码
                    │                           │                  │               │               │
                    │ 原始 Bayer/RGB            │ cv::Mat          │ CameraFrame    │ 条件变量      │ 阻塞等待
                    │ 缓冲区                    │ BGR 三通道       │ + 时间戳       │ 唤醒消费者    │ 100ms 超时
```

### 5.2 `captureLoop()` — 采集主循环

```cpp
void captureLoop() {
    while (running_) {                         // ① 循环直到通知退出
        // ② 尝试抢快锁 (20ms 超时)
        lock(camera_mutex_, 20ms)
        
        if (抢到了 且 已连接 且 handle 有效) {
            // ③ 从 SDK 取一帧（最多等 20ms）
            nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 20ms);
            
            if (成功) {
                data = processFrame(&stOutFrame);     // ④ 原始 → cv::Mat
                MV_CC_FreeImageBuffer(handle, ...);   // ⑤ 释放 SDK 缓冲区
                consecutive_sdk_errors_ = 0;          // ⑥ 重置错误计数
            }
            else if (NODATA) { /* 本轮无帧，不做处理 */ }
            else {
                错误计数++ >= 5 → markDisconnected()  // ⑦ 连续错误 → 判定断线
            }
        }
        
        if (未连接) { sleep(20ms); continue; }  // ⑧ 未连接时空转等待
        
        if (拿到帧) {
            更新心跳 last_frame_tick_ms_       // ⑨ 记录心跳
            pushFrame(std::move(frame));        // ⑩ 推入队列
        }
    }
}
```

#### 抓帧成功路径 (步骤③~⑥)

```
MV_CC_GetImageBuffer(handle, &stOutFrame, kGetImageBufferTimeoutMs)
```

| 参数 | 含义 |
|------|------|
| `handle` | 相机句柄 |
| `&stOutFrame` | **输出参数**，SDK 将帧数据写入此结构体 |
| `kGetImageBufferTimeoutMs` (20ms) | SDK 内部等待超时，20ms 内无帧则返回 `MV_E_NODATA` |

| 返回值 | 含义 | 处理 |
|--------|------|------|
| `MV_OK` | 拿到一帧 | 正常处理 |
| `MV_E_NODATA` | 超时无帧 | 跳过，本轮不做处理 |
| 其他错误 | SDK 异常 | 累计计数，≥5 次判定断线 |

#### 错误累积判定 (步骤⑦)

```
如果连续5次 GetImageBuffer 返回非 MV_OK 且非 NODATA 的错误
  → 认为相机已物理断开
  → markDisconnected() 记录断线时刻
  → daemonLoop 下轮检测到断线后触发 TryConnect()
```

> **为什么不每次错误都断线？** NODATA 只是暂时没有新帧，不是真正的错误。其他错误也可能偶发，连续 5 次才确认是**持续性问题**，避免误判。

### 5.3 `processFrame()` — 图像格式转换

```
输入：MV_FRAME_OUT *stOutFrame (海康 SDK 的原始帧结构)
输出：cv::Mat (OpenCV 三通道 BGR)
```

#### Bayer 格式处理（彩色工业相机最常见的格式）

工业相机传感器每个像素只有一种颜色（R/G/B），通过 **Bayer 模式**排列。需要插值还原成完整 RGB：

```
传感器原始输出:           OpenCV Bayer2BGR 后:
R  G  R  G               B G R  B G R
G  B  G  B      →        B G R  B G R
R  G  R  G               B G R  B G R
G  B  G  R               B G R  B G R
```

| SDK 格式 | OpenCV 转换码 | 说明 |
|----------|--------------|------|
| `BayerRG8` | `COLOR_BayerBG2BGR` | RG ↔ BG 交叉映射 |
| `BayerBG8` | `COLOR_BayerRG2BGR` | BG ↔ RG 交叉映射 |
| `BayerGB8` | `COLOR_BayerGR2BGR` | GB ↔ GR 交叉映射 |
| `BayerGR8` | `COLOR_BayerGB2BGR` | GR ↔ GB 交叉映射 |

> **为什么转换码看起来"反了"？** 海康 SDK 的 Bayer 命名基于**第一个像素的颜色**（如 RG = 第一行 R,G），而 OpenCV 的命名基于**2×2 块的布局**（如 BG = 第一行 B,G，第二行 G,R）。两者命名体系不同，需要交叉映射。

#### 小池化优化

```cpp
cv::Mat output_pool[kPoolCount];  // 5 个预分配 Mat
int pool_index = 0;               // 轮转索引

// processFrame 中：
int idx = pool_index;
cv::Mat &output = output_pool[idx];

// 检查 Mat 是否被外部持有（refcount > 1）
if (output.u->refcount > 1) {
    output = cv::Mat();  // 外部还在用 → 放弃复用，重新分配
}

output.create(height, width, CV_8UC3);
cv::cvtColor(bayerMat, output, cvt_code);
pool_index = (idx + 1) % 5;
return output;  // 返回的是引用而非拷贝
```

> **为什么用小池化？** 每帧都 `new cv::Mat` + `delete cv::Mat` 会频繁分配/释放内存。5 个预分配的 Mat 循环复用，只有在外部还持有时才重新分配。`refcount > 1` 检查是关键——确保不会覆盖正在被上层使用的帧。

#### RGB 格式处理

如果相机输出 `RGB8_Packed`，直接转换颜色空间：`RGB → BGR`（OpenCV 默认 BGR 顺序）。

### 5.4 `pushFrame()` — 帧入队

```cpp
void pushFrame(CameraFrame &&frame) {  // 右值引用：移动语义，避免拷贝
    {
        lock_guard<mutex> lock(queue_mutex_);
        if (frame_queue_.size() >= 2) {
            frame_queue_.pop();  // 丢弃最旧帧
        }
        frame_queue_.push(std::move(frame));
    }  // 锁在此处释放
    queue_cv_.notify_one();  // 唤醒一个等待在 getImage() 的消费者
}
```

| 参数 | 含义 |
|------|------|
| `frame` | 右值引用 `CameraFrame&&`，使用 `std::move` 传递，避免深拷贝 cv::Mat |

> **notify_one 在锁外调用**：这是个微优化。如果在锁内 notify，被唤醒的线程可能立即尝试获取同一个锁，造成"惊群"。

### 5.5 `getImage()` — 帧出队（外部接口）

```cpp
CameraFrame getImage() {
    unique_lock<mutex> lock(queue_mutex_);
    
    // 条件等待：队列非空 或 线程已停止
    bool has_data = queue_cv_.wait_for(lock, 100ms, [this] {
        return !frame_queue_.empty() || !running_;
    });
    
    if (has_data && !frame_queue_.empty()) {
        CameraFrame frame = std::move(frame_queue_.front());
        frame_queue_.pop();
        return frame;
    }
    
    return CameraFrame();  // 超时：返回空帧
}
```

**返回值：**
- 正常：包含 `cv::Mat` 和时间戳的 `CameraFrame`
- 超时/无数据：`CameraFrame()` 空对象（`frame.empty() == true`）

> **上层使用示例：**
> ```cpp
> CameraFrame f = camera.getImage();
> if (!f.frame.empty()) {
>     cv::imshow("preview", f.frame);
> }
> ```

### 5.6 `logRecoveryLatencyIfPending()` — 重连恢复延迟诊断

重连成功后**仅第一个帧**触发打印：

```
断线时刻 (t_disconnect_signal_ms_)
    ↓  ── 断线到 Pull 成功的耗时 ──
取流开始 (t_grab_started_ms_)
    ↓  ── Pull 到首帧的耗时 ──
首帧到达 (now)
    ↓
打印：open+OpenGrab=Xms, firstFrameAfterGrab=Yms, total=Zms
```

> 只打印一次（`first_frame_pending_` 用 `exchange(false)` 保证）。

---

## 6. hikcamera_reconnect.cpp — 守护自愈

这个文件实现了**自动检测 + 自动恢复**，是整个驱动"自愈能力"的核心。

### 6.1 `daemonLoop()` — 守护主循环

每 **100ms** 一轮，做两件事：

#### 第一部分：锁卡死诊断

```
尝试抢快锁 (20ms 超时)
  ├─ 抢到了 → 清除 wedge_detected_，清零计数器
  └─ 没抢到 → 计数器++
              连续 3 次没抢到 → wedge_detected_ = true，打 WARN 日志
              卡死超过 10 秒   → 升级为 ERROR 日志（建议重启进程）
```

> **这不会触发重连。** 锁卡死通常是采集线程在 SDK 调用中阻塞（比如 USB 线松动导致 `GetImageBuffer` 不返回），不是"断开"而是"卡住"。在进程内很难恢复，所以只是打日志。

#### 第二部分：连接状态与心跳检测

```
读取 isConnected（无锁）
读取 last_frame_tick_ms_（无锁）

计算心跳间隔 = now_ms - last_frame_tick_ms_

if (未连接 或 心跳超时 > 1000ms):
    if (心跳超时): markDisconnected()  // 记录断线时刻
    
    fresh_disconnect = was_connected_ && !connected  // 从连→断的变化沿
    fresh_stall      = stalled && !was_stalled_      // 从正常→卡顿的变化沿
    
    if (fresh_disconnect 或 fresh_stall):
        TryConnect(true)   // bypass_backoff=true：状态变化时立即重连
    else:
        TryConnect(false)  // 持续断线中：使用退避策略
```

> **变化沿检测**：`was_connected_` 和 `was_stalled_` 记录上一轮的状态，只有**刚发生的状态变化**才 `bypass_backoff=true`，避免每次轮询都重置退避计时器。

### 6.2 `TryConnect(bypass_backoff)` — 重连执行

```cpp
void TryConnect(bool bypass_backoff) {
    // ① 退避检查
    if (!bypass_backoff && 距上次重连 < reconnect_backoff_ms_) {
        return;  // 还没到重试时间
    }
    
    // ② 更新时间戳
    last_reconnect_attempt_ = now;
    
    // ③ 先关后开
    closeCamera();           // 清理旧句柄
    bool success = openCamera();  // 重新枚举+打开
    
    // ④ 更新退避参数
    if (success) {
        reconnect_backoff_ms_ = 200;  // 重置为初始值
        MAS_LOG_INFO("reconnect OK");
    } else {
        reconnect_backoff_ms_ = min(当前值 × 2, 500);  // 翻倍，上限 500ms
        // 日志节流：每 2 秒最多一条
        MAS_LOG_WARN("reconnect failed, next in {}ms", backoff);
    }
}
```

**`bypass_backoff` 参数：**

| 场景 | 值 | 含义 |
|------|-----|------|
| 刚断线/刚卡顿 | `true` | 立即重连，不等退避 |
| 持续断线中 | `false` | 按退避间隔重试 |

**退避策略：**
```
初始:        200ms
第1次失败:   400ms
第2次失败:   500ms (上限)
第N次失败:   500ms (持续重试)
一旦成功:    重置为 200ms
```

> **为什么上限只有 500ms？** 这是实时系统——不能等太久。500ms 已经比 daemon 轮询周期（100ms）长得多，足够避免 CPU 空转。

---

## 7. 三者协同全景图

### 7.1 文件 ↔ 角色映射

```
┌────────────────────────────────────────────────────────┐
│ hikcamera.hpp                                          │
│ 角色：蓝图 / 接口定义                                   │
│ 包含：类声明、所有成员变量、所有常量、所有函数签名        │
│ 依赖：MvCameraControl.h (海康 SDK), opencv, common_def  │
├────────────────────────────────────────────────────────┤
│ hikcamera.cpp                                          │
│ 角色：生命周期管理                                      │
│ 包含：构造/析构、openCamera、closeCamera、线程启停      │
│       辅助函数 (PrintDeviceInfo, ExceptionCallBack)     │
│ 依赖：hikcamera.hpp, YAML, MAS_LOG                      │
├────────────────────────────────────────────────────────┤
│ hikcamera_capture.cpp                                  │
│ 角色：图像采集管线                                      │
│ 包含：captureLoop、processFrame、pushFrame、getImage    │
│       logRecoveryLatencyIfPending                       │
│ 依赖：hikcamera.hpp, MAS_LOG                           │
├────────────────────────────────────────────────────────┤
│ hikcamera_reconnect.cpp                                │
│ 角色：健康监测 + 自动恢复                               │
│ 包含：daemonLoop、TryConnect                           │
│ 依赖：hikcamera.hpp, MAS_LOG                           │
└────────────────────────────────────────────────────────┘
```

### 7.2 时序全景

```
时间线 →

构造        openCamera()        正常运行                  USB松动           重连成功
 │              │                 │                       │                  │
 ├─ 加载YAML    ├─ 枚举设备       ├─ captureLoop 循环抓帧  ├─ SDK回调断线     ├─ openCamera 成功
 │              ├─ 创建句柄       ├─ daemonLoop 每100ms    ├─ markDisconnected├─ 重置退避
 │              ├─ 配置参数       │   检查心跳             ├─ daemonLoop检测   ├─ 首帧到达
 │              ├─ StartGrabbing  │                        │   到断线          ├─ 打印恢复延迟
 │              ├─ startThreads() │                        ├─ TryConnect()     ├─ 正常运行
 │              │                 │                        ├─ closeCamera
 │              │                 │                        ├─ openCamera (失败)
 │              │                 │                        ├─ backoff 200→400
 │              │                 │                        ├─ TryConnect()
 │              │                 │                        ├─ openCamera (成功)
```

### 7.3 关键设计决策总结

| 设计 | 为什么？ |
|------|---------|
| 三线程（主/capture/daemon） | 采集和守护不能阻塞主线程；守护必须独立于采集（采集可能卡死） |
| `timed_mutex` 而非 `mutex` | SDK 调用可能不返回，定时锁防止死锁 |
| fast/slow 两种锁超时 | 快路径（高频循环）不能等太久；慢路径（open/close）允许等 |
| 帧队列容量 2 | 只保留最新帧，不要积压过时数据 |
| Bayer→BGR 小池化 (5 个 Mat) | 减少内存分配，refcount 检查防止覆盖 |
| 连续错误 5 次才断线 | 去抖（debounce），避免单次偶发错误误判 |
| 退避 200→400→500ms | 快速响应 vs 避免 CPU 空转的平衡 |
| SDK 回调 + 心跳双重检测 | 双重保险，SDK 回调可能不触发（如 USB 松动） |
| 析构先停线程再关相机 | 防止线程使用已销毁的句柄 |

---

## 附录：快速查阅表

### SDK 调用速查

| SDK 函数 | 调用位置 | 作用 |
|----------|---------|------|
| `MV_CC_Initialize()` | `openCamera()` | 初始化 SDK（全局一次） |
| `MV_CC_Finalize()` | `~HikCamera()` | 反初始化 SDK |
| `MV_CC_EnumDevices()` | `openCamera()` | 枚举已连接的设备 |
| `MV_CC_CreateHandle()` | `openCamera()` | 为设备创建句柄 |
| `MV_CC_OpenDevice()` | `openCamera()` | 打开设备连接 |
| `MV_CC_CloseDevice()` | `closeCamera()` | 关闭设备连接 |
| `MV_CC_DestroyHandle()` | `closeCamera()` | 销毁句柄 |
| `MV_CC_StartGrabbing()` | `openCamera()` | 开始连续取流 |
| `MV_CC_StopGrabbing()` | `closeCamera()` | 停止取流 |
| `MV_CC_GetImageBuffer()` | `captureLoop()` | 获取一帧原始数据 |
| `MV_CC_FreeImageBuffer()` | `captureLoop()` | 释放帧缓冲区 |
| `MV_CC_GetFloatValue()` | `openCamera()` | 读取浮点参数范围 |
| `MV_CC_SetFloatValue()` | `openCamera()` | 设置浮点参数（曝光/增益） |
| `MV_CC_SetEnumValue()` | `openCamera()` | 设置枚举参数（触发模式） |
| `MV_CC_RegisterExceptionCallBack()` | `openCamera()` | 注册异常回调（断线通知） |

### 线程安全速查

| 资源 | 保护机制 | 访问者 |
|------|---------|--------|
| `handle` | `camera_mutex_` (timed) | openCamera, closeCamera, captureLoop, daemonLoop |
| `isConnected` | `atomic<bool>` | 所有函数（无锁读写） |
| `frame_queue_` | `queue_mutex_` + `queue_cv_` | pushFrame (写), getImage (读) |
| `running_` | `atomic<bool>` | startThreads, stopThreads, captureLoop, daemonLoop |
| `last_frame_tick_ms_` | `atomic<int64_t>` | captureLoop (写), daemonLoop (读) |
