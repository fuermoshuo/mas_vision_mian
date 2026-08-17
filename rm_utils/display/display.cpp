#include "display.hpp"
#include "mas_log.hpp"

namespace rm_utils
{

Display::Display()
{
    running_ = false;
    // 初始化 SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        MAS_LOG_ERROR("Failed to initialize SDL2: {}", SDL_GetError());
        return;
    }

    if (TTF_Init() < 0)
    {
        MAS_LOG_ERROR("Failed to initialize SDL_ttf: {}", TTF_GetError());
        return;
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
}

void Display::start(BS::thread_pool<> &pool)
{
    if (!running_)
    {
        running_        = true;
        display_future_ = pool.submit_task([this]() { display_thread_func(); });
        MAS_LOG_INFO("Display thread started");
    }
}

Display::~Display() { shutdown(); }

void Display::shutdown()
{
    // 只执行一次
    if (running_.exchange(false)) // 使用 exchange 原子地设为 false 并检查原值
    {
        if (display_future_.valid())
        {
            display_future_.get();
        }

        // 清理 SDL 资源
        for (auto &pair : sdl_contexts_)
        {
            for (auto &font_pair : pair.second.fonts)
            {
                if (font_pair.second) TTF_CloseFont(font_pair.second);
            }
            if (pair.second.texture) SDL_DestroyTexture(pair.second.texture);
            if (pair.second.renderer) SDL_DestroyRenderer(pair.second.renderer);
            if (pair.second.window) SDL_DestroyWindow(pair.second.window);
            for (auto &cache : pair.second.text_cache)
            {
                if (cache.texture) SDL_DestroyTexture(cache.texture);
            }
        }
        sdl_contexts_.clear();

        TTF_Quit();
        SDL_Quit();
    }
}

void Display::set_event_callback(SDL_Event_Callback callback, void *user_data)
{
    event_callback_           = callback;
    event_callback_user_data_ = user_data;
}

void Display::display_add(const std::string &window_name, const cv::Mat &img, const std::vector<DisplayText> &texts,
                          const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, uint32_t display_width,
                          uint32_t display_height)
{
    if (img.empty()) return;

    // FPS 限制: 30 FPS
    const auto kMinInterval = std::chrono::milliseconds(33);
    const auto now          = std::chrono::steady_clock::now();

    rigtorp::SPSCQueue<DisplayTask> *queue_ptr = nullptr;

    // 检查时间戳并获取/创建队列
    {
        std::lock_guard<std::mutex> lock(queues_mutex_);

        // 获取或创建队列
        if (window_queues_.find(window_name) == window_queues_.end())
        {
            // 每个窗口队列容量设为 2
            window_queues_[window_name] = std::make_unique<rigtorp::SPSCQueue<DisplayTask>>(2);
        }
        queue_ptr = window_queues_[window_name].get();

        // FPS 限流
        auto it = last_update_times_.find(window_name);
        if (it != last_update_times_.end())
        {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);
            if (duration.count() < kMinInterval.count())
            {
                return; // 未达到时间间隔，直接丢弃
            }
            it->second = now;
        }
        else
        {
            last_update_times_[window_name] = now;
        }
    }

    // 图像处理与发送
    DisplayTask task;
    task.window_name    = window_name;
    task.image          = img;
    task.texts          = texts;
    task.points         = points;
    task.lines          = lines;
    task.display_width  = display_width;
    task.display_height = display_height;

    if (queue_ptr)
    {
        bool success = queue_ptr->try_emplace(std::move(task));
    }
}

void Display::display_thread_func()
{
    while (running_)
    {
        // 处理 SDL 事件
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            // 如果有事件回调函数，调用它
            if (event_callback_ != nullptr)
            {
                // 调用回调函数，如果返回 true 表示事件已处理，不需要继续处理
                bool handled = event_callback_(event, event_callback_user_data_);
                if (handled)
                {
                    continue;
                }
            }

            // 默认处理窗口关闭事件
            if (event.type == SDL_QUIT)
            {
                // 找到对应的窗口并标记为关闭
                std::lock_guard<std::mutex> lock(queues_mutex_);
                for (auto &pair : sdl_contexts_)
                {
                    if (SDL_GetWindowID(pair.second.window) == event.window.windowID)
                    {
                        pair.second.closed = true;
                        break;
                    }
                }
            }
        };
        // 从队列获取任务
        std::vector<DisplayTask> tasks_to_show;
        {
            std::lock_guard<std::mutex> lock(queues_mutex_);
            for (auto &pair : window_queues_)
            {
                rigtorp::SPSCQueue<DisplayTask> *q = pair.second.get();
                DisplayTask                     *t = q->front();
                if (t != nullptr && !t->image.empty())
                {
                    // 如果窗口已关闭，跳过该窗口的任务
                    if (sdl_contexts_.count(pair.first) && sdl_contexts_[pair.first].closed)
                    {
                        q->pop();
                        continue;
                    }
                    tasks_to_show.push_back(std::move(*t));
                    q->pop();
                }
            }
        }

        // 渲染任务
        for (auto &task : tasks_to_show)
        {
            SDLContext &ctx = sdl_contexts_[task.window_name];

            // A. 如果窗口不存在，按传入的 display_width/height 创建窗口
            if (!ctx.window)
            {
                ctx.window = SDL_CreateWindow(task.window_name.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, task.display_width,
                                              task.display_height, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
                if (!ctx.window)
                {
                    MAS_LOG_ERROR("SDL_CreateWindow Error: {}", SDL_GetError());
                    continue;
                }

                // 创建渲染器，硬件加速
                ctx.renderer = SDL_CreateRenderer(ctx.window, -1, SDL_RENDERER_ACCELERATED);
                if (!ctx.renderer)
                {
                    MAS_LOG_ERROR("SDL_CreateRenderer Error: {}", SDL_GetError());
                    SDL_DestroyWindow(ctx.window);
                    ctx.window = nullptr;
                    continue;
                }

                ctx.width  = task.display_width;
                ctx.height = task.display_height;
            }

            // 如果要求的显示尺寸变化，调整窗口大小
            if (ctx.width != (int)task.display_width || ctx.height != (int)task.display_height)
            {
                SDL_SetWindowSize(ctx.window, task.display_width, task.display_height);
                ctx.width  = task.display_width;
                ctx.height = task.display_height;
            }

            // 创建或更新纹理
            if (!ctx.texture || (ctx.tex_w != task.image.cols || ctx.tex_h != task.image.rows))
            {
                if (ctx.texture) SDL_DestroyTexture(ctx.texture);

                ctx.texture = SDL_CreateTexture(ctx.renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, task.image.cols, task.image.rows);

                ctx.tex_w   = task.image.cols;
                ctx.tex_h  = task.image.rows;
            }

            if (ctx.texture)
            {
                cv::Mat upload_img;
                if (task.image.channels() == 1)
                {
                    cv::cvtColor(task.image, upload_img, cv::COLOR_GRAY2BGR);
                }
                else
                {
                    upload_img = task.image;
                }

                // 更新纹理
                if (upload_img.isContinuous())
                {
                    SDL_UpdateTexture(ctx.texture, nullptr, upload_img.data, (int)upload_img.step);
                }
                else
                {
                    cv::Mat continuous_img = upload_img.clone();
                    SDL_UpdateTexture(ctx.texture, nullptr, continuous_img.data, (int)continuous_img.step);
                }

                SDL_RenderClear(ctx.renderer);
                SDL_RenderCopy(ctx.renderer, ctx.texture, nullptr, nullptr);

                // 计算当前窗口相对于原图的缩放比例，用于坐标映射
                int win_w, win_h;
                SDL_GetWindowSize(ctx.window, &win_w, &win_h);
                float scale_x = static_cast<float>(win_w) / task.image.cols;
                float scale_y = static_cast<float>(win_h) / task.image.rows;

                // 1. 绘制直线
                for (const auto &line : task.lines)
                {
                    SDL_SetRenderDrawColor(ctx.renderer, line.color.r, line.color.g, line.color.b, line.color.a == 0 ? 255 : line.color.a);

                    int x1 = static_cast<int>(line.x1 * scale_x);
                    int y1 = static_cast<int>(line.y1 * scale_y);
                    int x2 = static_cast<int>(line.x2 * scale_x);
                    int y2 = static_cast<int>(line.y2 * scale_y);

                    if (line.thickness <= 1)
                    {
                        SDL_RenderDrawLine(ctx.renderer, x1, y1, x2, y2);
                    }
                    else
                    {
                        // 粗线实现：根据斜率在垂直方向或水平方向平移绘制
                        int dx = std::abs(x2 - x1);
                        int dy = std::abs(y2 - y1);
                        int t  = line.thickness;
                        if (dx > dy)
                        {
                            for (int i = -t / 2; i <= t / 2; i++)
                            {
                                SDL_RenderDrawLine(ctx.renderer, x1, y1 + i, x2, y2 + i);
                            }
                        }
                        else
                        {
                            for (int i = -t / 2; i <= t / 2; i++)
                            {
                                SDL_RenderDrawLine(ctx.renderer, x1 + i, y1, x2 + i, y2);
                            }
                        }
                    }
                }

                // 2. 绘制点
                for (const auto &point : task.points)
                {
                    SDL_SetRenderDrawColor(ctx.renderer, point.color.r, point.color.g, point.color.b, point.color.a == 0 ? 255 : point.color.a);
                    int      scaled_size = std::max(1, static_cast<int>(point.size * scale_x));
                    SDL_Rect rect = {static_cast<int>(point.x * scale_x) - scaled_size / 2, static_cast<int>(point.y * scale_y) - scaled_size / 2,
                                     scaled_size, scaled_size};
                    SDL_RenderFillRect(ctx.renderer, &rect);
                }

                // 3. 绘制文本
                for (const auto &text_task : task.texts)
                {
                    if (text_task.content.empty()) continue;

                    // 检查缓存
                    SDL_Texture *textTexture = nullptr;
                    int          tw = 0, th = 0;
                    bool         found = false;
                    for (auto &cache : ctx.text_cache)
                    {
                        if (cache.content == text_task.content && cache.size == text_task.size && cache.color.r == text_task.color.r &&
                            cache.color.g == text_task.color.g && cache.color.b == text_task.color.b && cache.thickness == text_task.thickness)
                        {
                            textTexture = cache.texture;
                            tw          = cache.w;
                            th          = cache.h;
                            found       = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        // 获取或加载对应字号的字体
                        TTF_Font *font = nullptr;
                        if (ctx.fonts.count(text_task.size))
                        {
                            font = ctx.fonts[text_task.size];
                        }
                        else
                        {
                            font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", text_task.size);
                            if (font)
                            {
                                ctx.fonts[text_task.size] = font;
                            }
                        }

                        if (font)
                        {
                            TTF_SetFontStyle(font, text_task.thickness > 1 ? TTF_STYLE_BOLD : TTF_STYLE_NORMAL);
                            SDL_Color final_color = text_task.color;
                            if (final_color.a == 0) final_color.a = 255;

                            SDL_Surface *textSurface = TTF_RenderText_Blended(font, text_task.content.c_str(), final_color);
                            if (textSurface)
                            {
                                textTexture = SDL_CreateTextureFromSurface(ctx.renderer, textSurface);
                                if (textTexture)
                                {
                                    tw = textSurface->w;
                                    th = textSurface->h;
                                    // 加入缓存
                                    if (ctx.text_cache.size() >= SDLContext::MAX_TEXT_CACHE)
                                    {
                                        SDL_DestroyTexture(ctx.text_cache.front().texture);
                                        ctx.text_cache.erase(ctx.text_cache.begin());
                                    }
                                    ctx.text_cache.push_back(
                                        {textTexture, tw, th, text_task.content, text_task.size, text_task.color, text_task.thickness});
                                }
                                SDL_FreeSurface(textSurface);
                            }
                        }
                    }

                    if (textTexture)
                    {
                        SDL_Rect textRect;
                        textRect.x = static_cast<int>(text_task.x * scale_x);
                        textRect.y = static_cast<int>(text_task.y * scale_y);
                        textRect.w = static_cast<int>(tw * scale_x);
                        textRect.h = static_cast<int>(th * scale_y);
                        SDL_RenderCopy(ctx.renderer, textTexture, nullptr, &textRect);
                    }
                }

                SDL_RenderPresent(ctx.renderer);
            }
        }

        // 避免死循环并降低 CPU 占用
        if (tasks_to_show.empty())
        {
            SDL_Delay(10);
        }
    }
}

} // namespace rm_utils