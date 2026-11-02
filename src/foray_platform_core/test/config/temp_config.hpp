#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>

namespace foray::platform::config::test {

/**
 * @brief 在系统临时目录创建配置测试目录，析构时递归删除。
 */
class TempConfigDir {
  public:
    TempConfigDir() {
        static std::atomic<unsigned> counter{0};
        path_ = std::filesystem::temp_directory_path() /
                ("foray_config_test_" + std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path_);
    }

    ~TempConfigDir() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    TempConfigDir(const TempConfigDir&) = delete;
    TempConfigDir& operator=(const TempConfigDir&) = delete;

    std::filesystem::path path() const {
        return path_;
    }

    /// @brief 写文件（可含子目录），返回文件路径。
    std::filesystem::path write(const std::string& name, const std::string& content) const {
        const std::filesystem::path file = path_ / name;
        std::filesystem::create_directories(file.parent_path());

        std::ofstream stream(file, std::ios::binary | std::ios::trunc);
        stream << content;
        return file;
    }

  private:
    std::filesystem::path path_;
};

} // namespace foray::platform::config::test
