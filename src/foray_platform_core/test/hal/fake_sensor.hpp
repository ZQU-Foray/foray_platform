#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "foray/platform/hal/camera.hpp"
#include "foray/platform/hal/frame_pool.hpp"
#include "foray/platform/hal/lidar.hpp"

namespace foray::platform::hal::test {

/**
 * @brief 测试用相机：实现 HAL 生命周期与读取契约，不需要 SDK/硬件。
 */
class FakeCamera final : public Camera {
  public:
    FakeCamera() : pool_(std::make_shared<FramePool<ImageFrame>>(2, 64)) {
        info_.vendor = "fake";
        info_.model = "fake-camera";
        info_.frame_id = "fake_camera";
    }

    Result<void> open() override {
        if (state_ != DeviceState::kClosed) {
            return failure(ErrorCode::kAlreadyOpen, "camera is already open");
        }
        state_ = DeviceState::kOpening;
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    Result<void> start() override {
        if (state_ != DeviceState::kOpen) {
            return failure(ErrorCode::kNotOpen, "camera is not open");
        }
        state_ = DeviceState::kStreaming;
        return Result<void>::success();
    }

    Result<void> stop() override {
        if (state_ != DeviceState::kStreaming) {
            return failure(ErrorCode::kNotOpen, "camera is not streaming");
        }
        state_ = DeviceState::kStopping;
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    Result<void> close() override {
        if (state_ == DeviceState::kClosed) {
            return Result<void>::success();
        }
        state_ = DeviceState::kClosed;
        return Result<void>::success();
    }

    Result<void> recover() override {
        if (state_ == DeviceState::kClosed) {
            return failure(ErrorCode::kNotOpen, "camera is closed");
        }
        state_ = DeviceState::kRecovering;
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    DeviceState state() const noexcept override {
        return state_;
    }

    SensorInfo info() const override {
        return info_;
    }

    SensorStats stats() const noexcept override {
        return stats_;
    }

    Result<void> read(FrameLease<ImageFrame>& out, time::Duration /*timeout*/) override {
        if (state_ != DeviceState::kStreaming) {
            return failure(ErrorCode::kNotOpen, "camera is not streaming");
        }
        FrameLease<ImageFrame> lease = pool_->try_acquire_latest();
        if (!lease) {
            ++stats_.timeouts;
            return failure(ErrorCode::kTimeout, "no frame available");
        }
        ++stats_.frames_delivered;
        stats_.last_sequence = lease->sequence;
        out = std::move(lease);
        return Result<void>::success();
    }

    Result<void> set_exposure(std::optional<double> microseconds) override {
        if (state_ == DeviceState::kClosed) {
            return failure(ErrorCode::kNotOpen, "camera is closed");
        }
        exposure_us_ = microseconds;
        return Result<void>::success();
    }

    Result<void> set_gain(std::optional<double> db) override {
        if (state_ == DeviceState::kClosed) {
            return failure(ErrorCode::kNotOpen, "camera is closed");
        }
        gain_db_ = db;
        return Result<void>::success();
    }

    Result<void> set_frame_rate(std::optional<double> hz) override {
        if (state_ == DeviceState::kClosed) {
            return failure(ErrorCode::kNotOpen, "camera is closed");
        }
        frame_rate_hz_ = hz;
        return Result<void>::success();
    }

    /// @brief 注入一帧，供测试读取。
    bool inject_frame(std::uint64_t sequence) {
        auto writer = pool_->try_acquire_writer();
        if (!writer.has_value()) {
            return false;
        }
        writer->frame().sequence = sequence;
        writer->frame().frame_id = info_.frame_id;
        writer->publish();
        ++stats_.frames_received;
        return true;
    }

    void mark_error() {
        state_ = DeviceState::kError;
    }

    const std::optional<double>& exposure_us() const noexcept {
        return exposure_us_;
    }

  private:
    static Result<void> failure(ErrorCode code, std::string message) {
        return Result<void>::failure(Error{code, std::move(message), 0});
    }

    std::shared_ptr<FramePool<ImageFrame>> pool_;
    DeviceState state_ = DeviceState::kClosed;
    SensorInfo info_;
    SensorStats stats_;
    std::optional<double> exposure_us_;
    std::optional<double> gain_db_;
    std::optional<double> frame_rate_hz_;
};

/**
 * @brief 测试用激光雷达。
 */
class FakeLidar final : public Lidar {
  public:
    FakeLidar() : pool_(std::make_shared<FramePool<PointCloudFrame>>(2, sizeof(PointXYZITL) * 16)) {
        info_.vendor = "fake";
        info_.model = "fake-lidar";
        info_.frame_id = "fake_lidar";
    }

    Result<void> open() override {
        if (state_ != DeviceState::kClosed) {
            return failure(ErrorCode::kAlreadyOpen, "lidar is already open");
        }
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    Result<void> start() override {
        if (state_ != DeviceState::kOpen) {
            return failure(ErrorCode::kNotOpen, "lidar is not open");
        }
        state_ = DeviceState::kStreaming;
        return Result<void>::success();
    }

    Result<void> stop() override {
        if (state_ != DeviceState::kStreaming) {
            return failure(ErrorCode::kNotOpen, "lidar is not streaming");
        }
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    Result<void> close() override {
        state_ = DeviceState::kClosed;
        return Result<void>::success();
    }

    Result<void> recover() override {
        if (state_ == DeviceState::kClosed) {
            return failure(ErrorCode::kNotOpen, "lidar is closed");
        }
        state_ = DeviceState::kRecovering;
        state_ = DeviceState::kOpen;
        return Result<void>::success();
    }

    DeviceState state() const noexcept override {
        return state_;
    }

    SensorInfo info() const override {
        return info_;
    }

    SensorStats stats() const noexcept override {
        return stats_;
    }

    Result<void> read(FrameLease<PointCloudFrame>& out, time::Duration /*timeout*/) override {
        if (state_ != DeviceState::kStreaming) {
            return failure(ErrorCode::kNotOpen, "lidar is not streaming");
        }
        FrameLease<PointCloudFrame> lease = pool_->try_acquire_latest();
        if (!lease) {
            ++stats_.timeouts;
            return failure(ErrorCode::kTimeout, "no point cloud available");
        }
        ++stats_.frames_delivered;
        out = std::move(lease);
        return Result<void>::success();
    }

    Result<void> read_imu(ImuSample& out, time::Duration /*timeout*/) override {
        if (state_ != DeviceState::kStreaming) {
            return failure(ErrorCode::kNotOpen, "lidar is not streaming");
        }
        if (!imu_.has_value()) {
            return failure(ErrorCode::kNoData, "no IMU sample available");
        }
        out = *imu_;
        imu_.reset();
        return Result<void>::success();
    }

    bool inject_cloud(std::uint64_t sequence, std::size_t count) {
        auto writer = pool_->try_acquire_writer();
        if (!writer.has_value() || count > writer->payload_size() / sizeof(PointXYZITL)) {
            return false;
        }
        writer->frame().points = reinterpret_cast<const PointXYZITL*>(writer->data());
        writer->frame().count = count;
        writer->frame().sequence = sequence;
        writer->frame().frame_id = info_.frame_id;
        writer->publish();
        ++stats_.frames_received;
        return true;
    }

    void inject_imu(ImuSample sample) {
        imu_ = sample;
    }

  private:
    static Result<void> failure(ErrorCode code, std::string message) {
        return Result<void>::failure(Error{code, std::move(message), 0});
    }

    std::shared_ptr<FramePool<PointCloudFrame>> pool_;
    DeviceState state_ = DeviceState::kClosed;
    SensorInfo info_;
    SensorStats stats_;
    std::optional<ImuSample> imu_;
};

} // namespace foray::platform::hal::test
