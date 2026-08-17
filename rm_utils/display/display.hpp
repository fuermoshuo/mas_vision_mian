#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "BS_thread_pool.hpp"
#include "SPSCQueue.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>

namespace rm_utils
{
struct DisplayText
{
    std::string content;
    int         x         = 0;
    int         y         = 0;
    int         size      = 24;
    SDL_Color   color     = {255, 255, 255, 255}; // 默认白色
    int         thickness = 1;                    // 1 为普通，>1 为加粗
};

struct DisplayPoint
{
    int       x     = 0;
    int       y     = 0;
    SDL_Color color = {255, 255, 255, 255};
    int       size  = 2; // 点的大小（像素半径）
};

struct DisplayLine
{
    int       x1        = 0;
    int       y1        = 0;
    int       x2        = 0;
    int       y2        = 0;
    int       thickness = 2;
    SDL_Color color     = {255, 255, 255, 255};
};

struct DisplayTask
{
    std::string               window_name;
    cv::Mat                   image;
    std::vector<DisplayText>  texts;          // 多个文本
    std::vector<DisplayPoint> points;         // 多个点
    std::vector<DisplayLine>  lines;          // 多条直线
    uint32_t                  display_width;  // 显示宽度
    uint32_t                  display_height; // 显示高度
};

// SDL 上下文结构体，用于存储每个窗口的 SDL 对象
struct SDLContext
{
    SDL_Window                         *window   = nullptr;
    SDL_Renderer                       *renderer = nullptr;
    SDL_Texture                        *texture  = nullptr; // 存储图像原图
    std::unordered_map<int, TTF_Font *> fonts; // 按字号缓存字体
    int                                 width  = 0; // 窗口
    int                                 height = 0;
    int                                 tex_w  = 0; // 纹理（原图）尺寸
    int                                 tex_h  = 0;
    bool                                closed = false; // 窗口是否已关闭

    // 文本纹理缓存
    struct CachedText
    {
        SDL_Texture *texture = nullptr;
        int          w       = 0;
        int          h       = 0;
        std::string  content;
        int          size      = 0;
        SDL_Color    color     = {0, 0, 0, 0};
        int          thickness = 1;
    };
    std::vector<CachedText> text_cache;
    static constexpr size_t MAX_TEXT_CACHE = 50;
};

// 事件回调函数类型定义
typedef bool (*SDL_Event_Callback)(const SDL_Event &event, void *user_data);

class Display
{
  public:
    // 禁止拷贝和赋值，确保单例唯一性
    Display(const Display &)            = delete;
    Display &operator=(const Display &) = delete;

    /**
     * @brief 获取全局唯一的 Display 实例
     * @return Display& 全局实例的引用
     */
    static Display &getInstance()
    {
        static Display instance;
        return instance;
    }

    /**
     * @brief 发送图像到显示队列
     * @param window_name 窗口名称
     * @param img 图像
     * @param texts 要显示的文本列表
     * @param points 要显示的点列表
     * @param lines 要显示的直线列表
     * @param display_width 显示宽度
     * @param display_height 显示高度
     */
    void display_add(const std::string &window_name, const cv::Mat &img,
                     const std::vector<DisplayText>  &texts  = {},
                     const std::vector<DisplayPoint> &points = {},
                     const std::vector<DisplayLine> &lines = {}, uint32_t display_width = 720,
                     uint32_t display_height = 540);

    /**
     * @brief 启动显示线程（使用线程池）
     * @param pool 线程池引用
     */
    void start(BS::thread_pool<> &pool);

    void shutdown();

    /**
     * @brief 设置事件回调函数
     * @param callback 回调函数指针
     * @param user_data 用户数据指针
     */
    void set_event_callback(SDL_Event_Callback callback, void *user_data);

  private:
    Display();
    ~Display();

    // 线程工作函数
    void display_thread_func();
    // 显示任务队列
    std::unordered_map<std::string, std::unique_ptr<rigtorp::SPSCQueue<DisplayTask>>>
        window_queues_;
    // 最后更新时间
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_update_times_;
    // 队列互斥锁
    std::mutex queues_mutex_;
    // 显示线程
    std::future<void> display_future_;
    // 运行标志
    std::atomic<bool> running_;
    // SDL 上下文存储
    std::unordered_map<std::string, SDLContext> sdl_contexts_;

    // 事件回调相关
    SDL_Event_Callback event_callback_           = nullptr;
    void              *event_callback_user_data_ = nullptr;
};

} // namespace rm_utils

#endif // _DISPLAY_H_