#pragma once

#include <string>
#include <utility>
#include <vector>

namespace foray::platform::param {

/**
 * @brief 参数系统错误码。
 */
enum class ParamErrorCode {
    /// @brief schema 定义本身非法（名称、默认值、边界等）。
    kInvalidSchema,
    /// @brief 重复定义同名参数。
    kDuplicateParam,
    /// @brief 配置中出现 schema 未声明的参数（通常是拼写错误）。
    kUnknownParam,
    /// @brief 值的类型与 schema 不符。
    kTypeMismatch,
    /// @brief 数值超出 schema 声明的范围。
    kOutOfRange,
    /// @brief 值不在 schema 声明的枚举集合中。
    kValueNotAllowed,
    /// @brief 热加载试图修改不可热加载的参数。
    kNotHotReloadable,
    /// @brief 参数源读取失败（文件不存在、解析错误等）。
    kSourceFailure,
    /// @brief 未完成初始加载就调用了热加载。
    kNotInitialized,
};

/// @brief 返回错误码名称。
const char* to_string(ParamErrorCode code) noexcept;

/**
 * @brief 单条参数错误。
 */
struct ParamError {
    ParamErrorCode code = ParamErrorCode::kSourceFailure;
    /// @brief 相关参数名或参数源名称；与具体参数无关时可为空。
    std::string name;
    /// @brief 人类可读的详细说明。
    std::string message;
};

/**
 * @brief 参数操作结果。
 *
 * 空错误列表表示成功。批量操作会收集全部错误后一次性返回，
 * 而不是遇到第一个错误就中止——这样一次热加载可以让用户看到所有配置问题。
 */
class ParamResult {
  public:
    /// @brief 构造成功结果。
    ParamResult() noexcept = default;

    static ParamResult success() {
        return ParamResult();
    }

    static ParamResult failure(ParamError error) {
        ParamResult result;
        result.errors_.push_back(std::move(error));
        return result;
    }

    static ParamResult failure(std::vector<ParamError> errors) {
        ParamResult result;
        result.errors_ = std::move(errors);
        return result;
    }

    bool ok() const noexcept {
        return errors_.empty();
    }

    explicit operator bool() const noexcept {
        return ok();
    }

    const std::vector<ParamError>& errors() const noexcept {
        return errors_;
    }

    /// @brief 将全部错误拼成多行文本，用于日志。
    std::string to_string() const;

  private:
    std::vector<ParamError> errors_;
};

} // namespace foray::platform::param
