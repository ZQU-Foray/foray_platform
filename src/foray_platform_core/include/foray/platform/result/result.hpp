#pragma once

#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "foray/platform/result/error.hpp"

namespace foray::platform {

/**
 * @brief 带错误值的返回类型。
 *
 * 公共 API 约定：
 * - 不抛异常；调用方先检查 `ok()`；
 * - `value()` 在失败时抛出 `std::bad_variant_access`，只应在 `ok()` 后调用；
 * - `error()` 在成功时抛出 `std::bad_variant_access`，只应在失败后调用；
 * - 成功路径不分配内存；失败路径只分配错误信息。
 *
 * `T` 可以是 move-only 类型（如 `std::unique_ptr`）。
 */
template <typename T> class Result {
  public:
    static_assert(!std::is_reference_v<T>, "Result<T> cannot hold a reference");

    Result(T value) : storage_(std::in_place_index<0>, std::move(value)) {
    }

    Result(Error error) : storage_(std::in_place_index<1>, std::move(error)) {
    }

    static Result success(T value) {
        return Result(std::move(value));
    }

    static Result failure(Error error) {
        return Result(std::move(error));
    }

    bool ok() const noexcept {
        return storage_.index() == 0;
    }

    explicit operator bool() const noexcept {
        return ok();
    }

    T& value() & {
        return std::get<0>(storage_);
    }

    const T& value() const& {
        return std::get<0>(storage_);
    }

    T&& value() && {
        return std::get<0>(std::move(storage_));
    }

    Error& error() & {
        return std::get<1>(storage_);
    }

    const Error& error() const& {
        return std::get<1>(storage_);
    }

  private:
    std::variant<T, Error> storage_;
};

/**
 * @brief 无返回值的 `Result`。
 */
template <> class Result<void> {
  public:
    Result() noexcept = default;

    Result(Error error) : error_(std::move(error)) {
    }

    static Result success() noexcept {
        return Result();
    }

    static Result failure(Error error) {
        return Result(std::move(error));
    }

    bool ok() const noexcept {
        return !error_.has_value();
    }

    explicit operator bool() const noexcept {
        return ok();
    }

    /// @brief 失败时返回错误；成功时抛出 `std::logic_error`。
    const Error& error() const& {
        if (!error_.has_value()) {
            throw std::logic_error("Result<void>::error() called on a successful result");
        }
        return *error_;
    }

  private:
    std::optional<Error> error_;
};

} // namespace foray::platform
