#include "viewer.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#if FORAY_HIKVISION_HAVE_SDL2
#include <SDL.h>
#endif

namespace foray::platform::driver::hikvision::tools {

namespace {

/// @brief RGB24 每像素字节数。
constexpr int kRgbChannels = 3;

/// @brief 窗口最长边上限：再大就等比缩小窗口，纹理仍按原始分辨率上传。
/// 取 1280 是因为常见笔记本与 1080p 桌面都能整窗显示，无需缩放查看细节。
constexpr int kMaxWindowSide = 1280;

/// @brief Bayer 2x2 块内各位置的通道角色，以及块内 R/B/两个 G 的下标。
///
/// 下标约定 `index = (y & 1) * 2 + (x & 1)`，即 0=(0,0) 1=(1,0) 2=(0,1) 3=(1,1)。
/// 角色取值 0=R、1=G、2=B。
struct BayerRoles {
    int roles[4];
    int red;
    int blue;
    int green_a;
    int green_b;
};

/// @brief 四种 8 位 Bayer 排列的块内布局；非 Bayer 格式返回全 -1。
BayerRoles bayer_roles(hal::PixelFormat format) noexcept {
    switch (format) {
    case hal::PixelFormat::kBayerRG8: // R G / G B
        return {{0, 1, 1, 2}, 0, 3, 1, 2};
    case hal::PixelFormat::kBayerGR8: // G R / B G
        return {{1, 0, 2, 1}, 1, 2, 0, 3};
    case hal::PixelFormat::kBayerGB8: // G B / R G
        return {{1, 2, 0, 1}, 2, 1, 0, 3};
    case hal::PixelFormat::kBayerBG8: // B G / G R
        return {{2, 1, 1, 0}, 3, 0, 1, 2};
    default:
        return {{-1, -1, -1, -1}, -1, -1, -1, -1};
    }
}

void convert_mono8(const hal::ImageFrame& frame, std::vector<std::uint8_t>& out) {
    const auto width = static_cast<std::size_t>(frame.width);
    const auto height = static_cast<std::size_t>(frame.height);
    const auto stride = static_cast<std::size_t>(frame.stride);
    std::uint8_t* dst = out.data();
    for (std::size_t y = 0; y < height; ++y) {
        const std::uint8_t* row = frame.data + y * stride;
        for (std::size_t x = 0; x < width; ++x) {
            const std::uint8_t value = row[x];
            *dst++ = value;
            *dst++ = value;
            *dst++ = value;
        }
    }
}

/**
 * @brief 块内插值去马赛克：本通道保留真实值，另两个通道取块内同色像素。
 *
 * 每个 2x2 块恰好含 1 个 R、2 个 G、1 个 B，所以色度是「半分辨率」的，
 * 但亮度通道没有被平滑——现场看视场与对焦足够，也避免了实现完整插值核。
 * 奇数宽/高时最后一列/行保持缓冲里的 0（调用方已清零），不留脏数据。
 */
void convert_bayer8(const hal::ImageFrame& frame, std::vector<std::uint8_t>& out) {
    const BayerRoles roles = bayer_roles(frame.pixel_format);
    const auto width = static_cast<std::size_t>(frame.width);
    const auto height = static_cast<std::size_t>(frame.height);
    const auto stride = static_cast<std::size_t>(frame.stride);

    for (std::size_t by = 0; by + 1 < height; by += 2) {
        const std::uint8_t* row0 = frame.data + by * stride;
        const std::uint8_t* row1 = row0 + stride;
        std::uint8_t* dst0 = out.data() + by * width * kRgbChannels;
        std::uint8_t* dst1 = dst0 + width * kRgbChannels;
        for (std::size_t bx = 0; bx + 1 < width; bx += 2) {
            const std::uint8_t block[4] = {row0[bx], row0[bx + 1], row1[bx], row1[bx + 1]};
            const std::uint8_t red = block[roles.red];
            const std::uint8_t blue = block[roles.blue];
            const std::uint8_t green =
                static_cast<std::uint8_t>((block[roles.green_a] + block[roles.green_b] + 1) / 2);

            for (int slot = 0; slot < 4; ++slot) {
                const int role = roles.roles[slot];
                std::uint8_t value_r = red;
                std::uint8_t value_g = green;
                std::uint8_t value_b = blue;
                if (role == 0) {
                    value_r = block[slot];
                } else if (role == 1) {
                    value_g = block[slot];
                } else {
                    value_b = block[slot];
                }
                // slot 的低位是块内 x 偏移，高位是 y 偏移。
                const std::size_t offset = bx + static_cast<std::size_t>(slot & 1);
                std::uint8_t* pixel =
                    (slot < 2 ? dst0 : dst1) + offset * kRgbChannels;
                pixel[0] = value_r;
                pixel[1] = value_g;
                pixel[2] = value_b;
            }
        }
    }
}

/// @brief 把一帧 8 位 Mono/Bayer 转成紧密排列的 RGB24；不支持的格式返回 false。
bool convert_to_rgb24(const hal::ImageFrame& frame, std::vector<std::uint8_t>& out) {
    if (frame.data == nullptr || frame.width == 0 || frame.height == 0) {
        return false;
    }
    const auto pixel_count =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    out.assign(pixel_count * kRgbChannels, 0);

    if (frame.pixel_format == hal::PixelFormat::kMono8) {
        convert_mono8(frame, out);
        return true;
    }
    if (bayer_roles(frame.pixel_format).red < 0) {
        return false;
    }
    convert_bayer8(frame, out);
    return true;
}

} // namespace

#if FORAY_HIKVISION_HAVE_SDL2

struct Viewer::Impl {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    std::vector<std::uint8_t> rgb;
    std::string title;
    int width = 0;
    int height = 0;
    bool initialized = false;

    ~Impl() {
        if (texture != nullptr) {
            SDL_DestroyTexture(texture);
        }
        if (renderer != nullptr) {
            SDL_DestroyRenderer(renderer);
        }
        if (window != nullptr) {
            SDL_DestroyWindow(window);
        }
        if (initialized) {
            // 工具只有一个 Viewer 实例；若将来要多窗口，这里要换成引用计数。
            SDL_Quit();
        }
    }

    Result<void> open(int frame_width, int frame_height) {
        if (window != nullptr) {
            return Result<void>(
                make_error(ErrorCode::kInternal, "预览窗口已经打开，不能重复初始化"));
        }
        if (frame_width <= 0 || frame_height <= 0) {
            return Result<void>(make_error(ErrorCode::kInvalidArgument, "预览窗口尺寸必须为正"));
        }

        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            return Result<void>(make_error(
                ErrorCode::kSdkUnavailable,
                std::string("SDL_Init 失败（没有可用显示设备？）: ") + SDL_GetError()));
        }
        initialized = true;

        const double scale = std::min(1.0, static_cast<double>(kMaxWindowSide) /
                                              static_cast<double>(std::max(frame_width, frame_height)));
        const int window_width = std::max(1, static_cast<int>(frame_width * scale));
        const int window_height = std::max(1, static_cast<int>(frame_height * scale));

        window = SDL_CreateWindow("foray_hikvision_dump", SDL_WINDOWPOS_UNDEFINED,
                                  SDL_WINDOWPOS_UNDEFINED, window_width, window_height,
                                  SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (window == nullptr) {
            return Result<void>(make_error(ErrorCode::kInternal,
                                           std::string("SDL_CreateWindow 失败: ") + SDL_GetError()));
        }

        // 不开 PRESENTVSYNC：让显示不反向限流取帧节奏（126 Hz 的源被 60 Hz 拖慢会让
        // 帧池覆盖与丢帧统计都变得难解释）。代价是可能出现撕裂，预览场景可接受。
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (renderer == nullptr) {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
        }
        if (renderer == nullptr) {
            return Result<void>(make_error(ErrorCode::kInternal,
                                           std::string("SDL_CreateRenderer 失败: ") + SDL_GetError()));
        }

        // 让缩放保持宽高比：窗口可以被自由拉伸，但画面不该被非等比拉扁——现场用这个窗口
        // 判断「视场对不对」时，比例失真会直接导致误判。SDL 会在两侧自动留黑边。
        if (SDL_RenderSetLogicalSize(renderer, frame_width, frame_height) != 0) {
            return Result<void>(make_error(
                ErrorCode::kInternal,
                std::string("SDL_RenderSetLogicalSize 失败: ") + SDL_GetError()));
        }

        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
                                    frame_width, frame_height);
        if (texture == nullptr) {
            return Result<void>(make_error(
                ErrorCode::kInternal, std::string("SDL_CreateTexture 失败: ") + SDL_GetError()));
        }

        width = frame_width;
        height = frame_height;
        return Result<void>();
    }

    bool show(const hal::ImageFrame& frame) {
        if (texture == nullptr) {
            return false;
        }
        if (static_cast<int>(frame.width) != width || static_cast<int>(frame.height) != height) {
            // 分辨率变化（ROI 或 binning 被改）不在预览期间重新建纹理，直接丢弃，
            // 避免 SDL 纹理在渲染中途重建导致的闪断。
            return false;
        }
        if (!convert_to_rgb24(frame, rgb)) {
            return false;
        }
        if (SDL_UpdateTexture(texture, nullptr, rgb.data(), width * kRgbChannels) != 0) {
            return false;
        }
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
        return true;
    }

    bool pump() {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                return false;
            }
            if (event.type == SDL_KEYDOWN) {
                const SDL_Keycode key = event.key.keysym.sym;
                if (key == SDLK_ESCAPE || key == SDLK_q) {
                    return false;
                }
            }
        }
        return true;
    }

    void set_title(const std::string& text) {
        if (window == nullptr || text == title) {
            return;
        }
        title = text;
        SDL_SetWindowTitle(window, title.c_str());
    }
};

#else

struct Viewer::Impl {
    bool initialized = false;

    Result<void> open(int, int) {
        return Result<void>(make_error(
            ErrorCode::kNotSupported,
            "本次构建未启用 SDL2（配置期找不到 SDL2）：重新配置并安装 libsdl2-dev 后可用 --view"));
    }

    bool show(const hal::ImageFrame&) {
        return false;
    }

    bool pump() {
        return false;
    }

    void set_title(const std::string&) {}
};

#endif

Viewer::Viewer() : impl_(std::make_unique<Impl>()) {}

Viewer::~Viewer() = default;

Result<void> Viewer::open(int width, int height) {
    return impl_->open(width, height);
}

bool Viewer::show(const hal::ImageFrame& frame) {
    return impl_->show(frame);
}

bool Viewer::pump() {
    return impl_->pump();
}

void Viewer::set_title(const std::string& title) {
    impl_->set_title(title);
}

} // namespace foray::platform::driver::hikvision::tools
