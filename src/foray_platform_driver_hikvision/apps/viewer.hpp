#ifndef FORAY_PLATFORM_DRIVER_HIKVISION_TOOLS_VIEWER_HPP
#define FORAY_PLATFORM_DRIVER_HIKVISION_TOOLS_VIEWER_HPP

#include <cstdint>
#include <memory>
#include <string>

#include "foray/platform/hal/frame.hpp"
#include "foray/platform/hal/sensor_stats.hpp"
#include "foray/platform/result/error.hpp"
#include "foray/platform/result/result.hpp"

namespace foray::platform::driver::hikvision::tools {

/**
 * @brief 最小的 SDL2 实时预览窗口，只给 `foray_hikvision_dump --view` 用。
 *
 * 显示链路刻意做得很薄：把 8 位 Mono/Bayer 帧就地转成 RGB24，贴到一张流式纹理
 * 上再呈现。**不做**色彩校正、去马赛克滤波或缩放插值——这个窗口是给现场
 * 「相机到底出没出图、视场对不对」用的，不是成像质量评估工具。
 *
 * 编译期未找到 SDL2 时（`FORAY_HIKVISION_HAVE_SDL2=0`）退化为退化实现：
 * `open()` 返回 `kNotSupported`，其余方法无副作用。
 *
 * @note 必须在主线程构造与销毁：SDL 的视频初始化要求主线程。
 */
class Viewer {
public:
    Viewer();
    ~Viewer();

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;
    Viewer(Viewer&&) = delete;
    Viewer& operator=(Viewer&&) = delete;

    /**
     * @brief 按帧尺寸建窗口、渲染器与纹理。
     *
     * @param width  帧宽（像素），必须 > 0
     * @param height 帧高（像素），必须 > 0
     * @return 失败时带具体原因（SDL 未启用、初始化失败、无显示设备等）
     */
    Result<void> open(int width, int height);

    /**
     * @brief 转换并呈现一帧。
     *
     * 只接受 `kMono8` 与四种 8 位 Bayer；其它格式画成黑屏并返回 false，
     * 让调用方把不支持的格式计入统计而不是崩在转换里。
     *
     * @return 是否真的画上了
     */
    bool show(const hal::ImageFrame& frame);

    /**
     * @brief 处理待处理事件。
     *
     * @return false 表示用户关窗或按了 ESC/q，调用方应结束循环
     */
    bool pump();

    /**
     * @brief 更新窗口标题（建议放帧率与计数）。
     *
     * 每帧调用一次即可：内部只在文本变化时才真的调 `SDL_SetWindowTitle`。
     */
    void set_title(const std::string& title);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace foray::platform::driver::hikvision::tools

#endif // FORAY_PLATFORM_DRIVER_HIKVISION_TOOLS_VIEWER_HPP
