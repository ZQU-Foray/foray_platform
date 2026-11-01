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

/// 真实实现：`fork` + `execvp`，stdout / stderr 用 `poll` 并发读到底。
///
/// 该调用**阻塞到子进程关闭两个输出管道为止**，没有超时——调用方要么给会被
/// 杀掉的短命令（`nmcli` 之类），要么自己加看护；不要用它跑长驻进程。
class ExecProcessRunner final : public ProcessRunner {
  public:
    Result<ProcessOutput> run(const std::vector<std::string>& argv) override;
};

} // namespace foray::platform::net
