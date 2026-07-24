# mas_log 模块逐行详解

> 适用读者：C++ 基础薄弱、对标准库和 spdlog 不熟悉的初学者。

---

## 一、先搞清楚：这套代码在干什么？

想象你要写日记：
- 有些事很重要（"今天面试了"）→ 要写在日记本上
- 有些事很琐碎（"今天剪了指甲"）→ 嘴上说说就行了，不用记

程序也一样。运行时会产出大量信息：
- "系统启动了"、"配置文件加载失败" → **重要，需要记录下来**
- "变量 x 的值是 3" → **调试用的，平常不用看**

**日志系统** 就是帮你分类记录这些信息的东西。`mas_log` 这个模块就是把 C++ 生态里最流行的日志库 **spdlog** 封装成项目自己的接口，方便整个项目使用。

---

## 二、`mas_log.hpp` 逐行讲解

### 第 1-2 行：头文件保护
```cpp
#ifndef _MAS_LOG_H_
#define _MAS_LOG_H_
```
- `#ifndef` = "if not defined"，如果 `_MAS_LOG_H_` 还没被定义过，就执行下面的代码。
- 作用：防止这个头文件被多次 `#include` 导致重复定义。
- 文件末尾的 `#endif` 和它配对，形成"只执行一次"的代码块。

### 第 4-7 行：include 头文件
```cpp
#include "spdlog/spdlog.h"
#include <memory>
#include <spdlog/common.h>
#include <string>
```
| 头文件 | 作用 |
|--------|------|
| `spdlog/spdlog.h` | spdlog 日志库的主头文件，提供日志功能 |
| `spdlog/common.h` | spdlog 的公共定义（比如日志级别 `level_enum`） |
| `<memory>` | C++ 标准库，提供智能指针 `std::shared_ptr` |
| `<string>` | C++ 标准库，提供字符串类 `std::string` |

### 第 9-20 行：5 个日志宏

```cpp
#define MAS_LOG_TRACE(...) \
    SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::trace, __VA_ARGS__)
```

**逐个拆解**：

| 符号 | 含义 |
|------|------|
| `#define` | 宏定义，编译前的文本替换 |
| `MAS_LOG_TRACE` | 宏的名字，给项目用的 |
| `(...)` | 可变参数，类似 `printf` 的 `...`，可以传任意数量参数 |
| `\` | 行续接符，宏定义太长换行用 |
| `SPDLOG_LOGGER_CALL` | spdlog 提供的宏，向指定 logger 写一条日志 |
| `rm_utils::MasLog::get_logger()` | 获取全局唯一的 logger 对象 |
| `spdlog::level::trace` | 日志级别 = trace（最低、最详细） |
| `__VA_ARGS__` | 替换成调用时传入的实际参数 |

**5 个宏的唯一区别就是级别不同**，从低到高：

```
MAS_LOG_TRACE    → spdlog::level::trace      最详细，几乎什么都记
MAS_LOG_DEBUG    → spdlog::level::debug      调试信息
MAS_LOG_INFO     → spdlog::level::info       普通运行信息
MAS_LOG_WARN     → spdlog::level::warn       警告（有小问题但不影响运行）
MAS_LOG_ERROR    → spdlog::level::err        错误（功能受影响）
MAS_LOG_CRITICAL → spdlog::level::critical   严重错误（程序可能要崩了）
```

比如你写：
```cpp
MAS_LOG_INFO("线程池创建成功，共 {} 个线程", 6);
```
宏展开后变成：
```cpp
SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::info,
                   "线程池创建成功，共 {} 个线程", 6);
```

### 第 22-24 行：命名空间与类声明

```cpp
namespace rm_utils
{
class MasLog
{
```
- **`namespace rm_utils`**：命名空间，相当于给代码加了个"姓"。`MasLog` 全名是 `rm_utils::MasLog`，防止和别人写的 `MasLog` 冲突。
- **`class MasLog`**：声明一个类，这个类不用于创建对象，只提供**静态工具方法**。

### 第 27-32 行：`init()` 函数

```cpp
static void init(const std::string        &log_path     = "logs/mas_vision.log",
                 spdlog::level::level_enum filelevel    = spdlog::level::debug,
                 spdlog::level::level_enum consolelevel = spdlog::level::warn);
```

| 关键字 | 含义 |
|--------|------|
| `static` | 这个方法属于**类本身**，不需要创建对象就能调用：`MasLog::init(...)` |
| `const std::string &` | 引用传参，避免拷贝字符串（高效）。`&` 读作"别名" |
| `= "logs/mas_vision.log"` | 默认参数，如果不传就用这个值 |

三个参数的作用：
1. `log_path` — 日志文件存哪
2. `filelevel` — 写进文件的日志最低级别（默认 debug：比较详细）
3. `consolelevel` — 显示到终端的日志最低级别（默认 warn：只有警告以上才显示）

### 第 38-45 行：`get_logger()` 函数

```cpp
static std::shared_ptr<spdlog::logger> get_logger()
{
    return logger_;
}
```

- **`std::shared_ptr<spdlog::logger>`**：智能指针。
  - 普通指针 `spdlog::logger *p` 需要手动 `delete`，容易忘导致内存泄漏。
  - 智能指针 `std::shared_ptr<spdlog::logger>` 会自动释放内存，"最后一个使用者消失时自动 delete"。
  - `shared` 意思是多个地方可以**共享**同一个对象。

- 这个函数就是把私有的 `logger_` 暴露给外部（但通过宏间接使用，你不会直接调用它）。

### 第 47-48 行：私有成员

```cpp
private:
    static std::shared_ptr<spdlog::logger> logger_;
```

- **`private`**：只有类内部能访问。
- **`static`** 成员变量：整个程序**只有一份**，不属于任何对象。
- `logger_` 就是那个全局唯一的日志记录器。

---

## 三、`mas_log.cpp` 逐行讲解

### 第 1-5 行：include

```cpp
#include "mas_log.hpp"
#include <spdlog/async.h>              // spdlog 的异步日志功能
#include <spdlog/sinks/basic_file_sink.h>   // sink = 输出目的地：文件
#include <spdlog/sinks/stdout_color_sinks.h> // sink = 输出目的地：彩色控制台
```

**什么是 sink？**
sink（汇）就是日志的"出口"。可以同时有多个出口：
- `basic_file_sink` → 写到文件
- `stdout_color_sink` → 打印到终端（带颜色）

### 第 7-11 行：include 标准库

```cpp
#include <chrono>       // 时间相关（获取当前时间）
#include <filesystem>   // 文件系统操作（创建目录、检查文件是否存在）
#include <iomanip>      // 格式化输出（put_time）
#include <sstream>      // 字符串流（拼接字符串用）
#include <vector>       // 动态数组容器
```

### 第 16 行：静态成员初始化

```cpp
std::shared_ptr<spdlog::logger> MasLog::logger_ = nullptr;
```

- 静态成员必须在类**外部**（.cpp 里）定义一次。
- `= nullptr` 初始化为空指针，等 `init()` 调用后才指向真正的 logger。

### 第 18-75 行：`init()` 函数实现

#### 整体结构

```cpp
void MasLog::init(const std::string &log_path,
                  spdlog::level::level_enum filelevel,
                  spdlog::level::level_enum consolelevel)
{
    try
    {
        // ... 所有初始化逻辑 ...
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::fprintf(stderr, "Log initialization failed: %s\n", ex.what());
    }
}
```

- **`try { ... } catch (...) { ... }`**：异常处理。try 块里的代码如果出错，不会让整个程序崩溃，而是跳到 catch 块里处理。
- **`spdlog::spdlog_ex`**：spdlog 库自己的异常类型。
- **`stderr`**：标准错误输出（终端），即使日志系统初始化失败，也至少能在终端看到报错。

#### 步骤 ①：生成带时间戳的文件名（第 23-37 行）

```cpp
std::filesystem::path p(log_path);
```
把 `"logs/mas_vision.log"` 这个字符串解析成一个路径对象，自动处理 `/` 和 `\` 的差异。

```cpp
auto parent_path = p.parent_path();  // 获取父目录 → "logs"
if (!parent_path.empty() && !std::filesystem::exists(parent_path))
{
    std::filesystem::create_directories(parent_path);  // 如果 logs/ 目录不存在，创建它
}
```
层层解析：路径 → 父目录 → 不存在就创建。

```cpp
auto now       = std::chrono::system_clock::now();        // 获取当前时间点
auto in_time_t = std::chrono::system_clock::to_time_t(now); // 转成 C 语言时间格式
std::stringstream ss;
ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H:%M");  // 格式化成 "2026-07-17_14:30"
```

| 代码 | 解释 |
|------|------|
| `system_clock::now()` | 获取当前系统时间 |
| `to_time_t(...)` | 转成 `time_t`（秒数，C 语言的时间表示法） |
| `localtime(&in_time_t)` | 把秒数转成"年月日时分秒"的结构体 |
| `put_time(..., "%Y-%m-%d_%H:%M")` | 按格式输出 → `2026-07-17_14:30` |
| `stringstream` | 类似 `StringBuilder`，把多段文字拼起来 |

```cpp
std::string filename       = p.stem().string() + "_" + ss.str() + p.extension().string();
```
- `p.stem()` → `"mas_vision"`（文件名去掉扩展名）
- `p.extension()` → `".log"`（扩展名）
- 拼接结果：`"mas_vision_2026-07-17_14:30.log"`

```cpp
std::string final_log_path = (parent_path / filename).string();
```
- `parent_path / filename` → `"logs" / "mas_vision_2026-07-17_14:30.log"`
- 最终路径：`"logs/mas_vision_2026-07-17_14:30.log"`

#### 步骤 ②：初始化异步线程池（第 40 行）

```cpp
spdlog::init_thread_pool(65536, 1);
```

| 参数 | 含义 |
|------|------|
| `65536` | 缓冲区大小（字节），日志先存这里 |
| `1` | 后台线程数，1 个线程专门负责写日志 |

**为什么要异步？**
写文件是慢操作（磁盘 I/O）。如果主线程自己写日志，程序会卡顿。用后台线程写，主线程只要把日志消息丢进队列就能继续干活。

#### 步骤 ③：创建控制台 sink（第 43-44 行）

```cpp
auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
console_sink->set_level(consolelevel);
```

- **`std::make_shared<T>()`**：创建一个 `T` 类型的对象，返回 `shared_ptr`。等价于 `new T()`，但是安全的。
- **`stdout_color_sink_st`**：spdlog 内置的控制台输出 sink（单线程版本 `_st`），不同级别显示不同颜色。
- **`->`**：通过智能指针访问成员，和普通指针的 `->` 一样。

#### 步骤 ④：创建文件 sink（第 47-48 行）

```cpp
auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(final_log_path, true);
file_sink->set_level(filelevel);
```

| 参数 | 含义 |
|------|------|
| `final_log_path` | 要写入的文件路径 |
| `true` | truncate 模式，每次都创建新文件（而不是追加到旧文件） |

#### 步骤 ⑤：合并 sink（第 51 行）

```cpp
std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
```
- **`std::vector<T>`**：动态数组，可以往里加元素。
- **`spdlog::sink_ptr`**：sink 的智能指针类型。
- **`{console_sink, file_sink}`**：初始化列表，创建 vector 时直接放入两个元素。

#### 步骤 ⑥：创建异步 logger（第 54-56 行）

```cpp
logger_ = std::make_shared<spdlog::async_logger>(
    "mas_logger", sinks.begin(), sinks.end(), spdlog::thread_pool(),
    spdlog::async_overflow_policy::overrun_oldest);
```

| 参数 | 含义 |
|------|------|
| `"mas_logger"` | logger 的名字（标识用） |
| `sinks.begin(), sinks.end()` | 要用的 sink 列表的迭代器范围 |
| `spdlog::thread_pool()` | 使用步骤②创建的线程池 |
| `overrun_oldest` | 如果日志太多来不及写，**丢弃最旧的**，保留最新的 |

#### 步骤 ⑦：设置日志级别（第 59-60 行）

```cpp
auto min_level = (filelevel < consolelevel) ? filelevel : consolelevel;
logger_->set_level(min_level);
```
- `? :` 是三元运算符：`条件 ? 真时的值 : 假时的值`
- 取文件级别和控制台级别中**更低的那个**（低 = 更详细），因为 logger 总体级别要取最宽松的，然后靠各自 sink 的级别来分别控制输出。

#### 步骤 ⑧：设置输出格式（第 62 行）

```cpp
logger_->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
```

| 占位符 | 含义 | 示例 |
|--------|------|------|
| `%H:%M:%S` | 时:分:秒 | `14:30:05` |
| `%e` | 毫秒 | `.123` |
| `%^ ... %$` | 包裹的部分显示颜色 | |
| `%l` | 日志级别 | `info`, `warn`, `error` |
| `%s` | 源文件名 | `main.cpp` |
| `%#` | 行号 | `58` |
| `%v` | 实际的日志内容 | `Mas Vision System Start` |

最终输出效果：
```
[14:30:05.123] [info] [main.cpp:58] Mas Vision System Start
```

#### 步骤 ⑨：刷新策略（第 64-65 行）

```cpp
spdlog::flush_every(std::chrono::seconds(3));   // 每 3 秒自动刷盘
logger_->flush_on(spdlog::level::warn);          // 遇到 warn 及以上级别立即刷盘
```

- **`flush`**：把内存缓冲区的数据真正写到磁盘上。写文件时操作系统会缓存一段时间再写入（为了效率），`flush` 强制它立刻写。
- 作用：如果程序突然崩溃，`warn` 级别的关键日志已经落盘不会丢失。

#### 步骤 ⑩：注册为全局 logger（第 67 行）

```cpp
spdlog::set_default_logger(logger_);
```

这样即使有代码直接用 spdlog 的 `spdlog::info()` 而不是 `MAS_LOG_INFO()`，也能输出到同一个地方。

---

## 四、总结：一张图看懂整体架构

```
代码调用                         数据流向                        输出
─────────                       ────────                       ────

MAS_LOG_INFO("hello")  ──→  SPDLOG_LOGGER_CALL  ──→  async_logger  ──┬── file_sink   ──→ logs/mas_vision_xxx.log
                                                      (异步, 后台线程)  │
MAS_LOG_WARN("oops")   ──→  SPDLOG_LOGGER_CALL  ──→  async_logger  ──┤
                                                                      │
MAS_LOG_ERROR("!!!")   ──→  SPDLOG_LOGGER_CALL  ──→  async_logger  ──┴── color_console_sink ──→ 终端(彩色)
```

---

## 五、关键 C++ 概念速查表

| 语法 | 简单解释 | 类比 |
|------|---------|------|
| `::` | 作用域解析符，"谁谁谁里面的" | `rm_utils::MasLog` = "rm_utils 家族里的 MasLog" |
| `&` | 引用，别名 | 不复制整个东西，只拿它的"代号" |
| `*` | 指针 | 存储一个内存地址 |
| `->` | 通过指针/智能指针访问成员 | `p->func()` = `(*p).func()` |
| `std::shared_ptr<T>` | 共享智能指针 | 多个主人共有一件物品，最后一个主人走的时候自动扔掉 |
| `std::make_shared<T>(...)` | 创建 shared_ptr | 比 `new T(...)` 安全 |
| `std::vector<T>` | 动态数组 | Python 的 `list` |
| `std::string` | 字符串 | Python 的 `str` |
| `static` (成员) | 属于类，不属于对象 | 全班共用的饮水机（而不是每人一个水杯） |
| `namespace` | 命名空间 | 姓氏，防重名 |
| `const` | 不可修改 | 只读 |
| `auto` | 自动推断类型 | 让编译器猜类型 |
| `try/catch` | 异常处理 | if 出错 → 跳到 catch |
