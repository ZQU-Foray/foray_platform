#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <string_view>

#include "foray/platform/param/value.hpp"

namespace foray::platform::param {

/**
 * @brief 参数名到值的映射。
 *
 * 使用透明比较器 `std::less<>`，查询时可直接传 `std::string_view`，
 * 不会产生临时 `std::string`。
 */
using ParamValueMap = std::map<std::string, ParamValue, std::less<>>;

/**
 * @brief 不可变参数快照。
 *
 * 一次控制周期内应只获取一次快照并在周期内复用，以保证同一周期看到的
 * 参数彼此一致（不会出现一半旧值、一半新值）。
 *
 * 读取接口不加锁；`find()` 不分配内存，可用于实时线程。
 * 隐式转换 getter 在缺失或类型不符时返回调用方提供的默认值。
 */
class ParamSnapshot {
  public:
    ParamSnapshot(ParamValueMap values, std::uint64_t generation);

    /**
     * @brief 按名称查找。
     * @return 指向内部值的指针；不存在返回 `nullptr`。指针在快照存活期间有效。
     */
    const ParamValue* find(std::string_view name) const noexcept;

    bool get_bool(std::string_view name, bool default_value) const noexcept;

    std::int64_t get_int(std::string_view name, std::int64_t default_value) const noexcept;

    /// @brief 读取数值；`int` 自动宽化为 `double`。
    double get_double(std::string_view name, double default_value) const noexcept;

    std::string get_string(std::string_view name, std::string default_value) const;

    const ParamValueMap& values() const noexcept {
        return values_;
    }

    /// @brief 快照代号；每次成功修改参数后递增。可用于检测配置是否变化。
    std::uint64_t generation() const noexcept {
        return generation_;
    }

  private:
    ParamValueMap values_;
    std::uint64_t generation_;
};

} // namespace foray::platform::param
