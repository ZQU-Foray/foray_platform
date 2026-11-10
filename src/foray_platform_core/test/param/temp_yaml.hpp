#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace foray::platform::param::test {

/**
 * @brief 在系统临时目录创建 YAML 文件，析构时删除。
 */
class TempYaml {
  public:
    explicit TempYaml(std::string content) {
        static std::atomic<unsigned> counter{0};
        path_ = std::filesystem::temp_directory_path() /
                ("foray_platform_test_" + std::to_string(counter.fetch_add(1)) + ".yaml");
        write(std::move(content));
    }

    ~TempYaml() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    TempYaml(const TempYaml&) = delete;
    TempYaml& operator=(const TempYaml&) = delete;

    void write(std::string content) const {
        std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
        stream << content;
    }

    std::string path() const {
        return path_.string();
    }

  private:
    std::filesystem::path path_;
};

} // namespace foray::platform::param::test
