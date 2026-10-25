#pragma once

#include <string>
#include <vector>

#include "foray/platform/result/result.hpp"

namespace foray::platform::net {

/// 子进程执行结果；`exit_code` 为 127 表示程序不存在。
struct ProcessOutput {
    int exit_code = -1;
    std::string out;
    std::string err;
};

/// 命令执行抽象：真实实现走 fork+exec，测试用 FakeProcessRunner 记录调用。
class ProcessRunner {
  public:
    virtual ~ProcessRunner() = default;
    virtual Result<ProcessOutput> run(const std::vector<std::string>& argv) = 0;
};

class ExecProcessRunner final : public ProcessRunner {
  public:
    Result<ProcessOutput> run(const std::vector<std::string>& argv) override;
};

} // namespace foray::platform::net
