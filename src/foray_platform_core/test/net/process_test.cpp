#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "foray/platform/net/process.hpp"

namespace foray::platform::net {
namespace {

/// @brief 用 `sh -c` 跑一段脚本，返回原始结果。
Result<ProcessOutput> run_script(const std::string& script) {
    ExecProcessRunner runner;
    return runner.run({"/bin/sh", "-c", script});
}

TEST(ExecProcessRunner, CapturesStdoutAndExitCode) {
    const auto result = run_script("printf hello");

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().out, "hello");
    EXPECT_EQ(result.value().exit_code, 0);
}

TEST(ExecProcessRunner, CapturesStderrAndNonZeroExit) {
    const auto result = run_script("printf oops 1>&2; exit 3");

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().err, "oops");
    EXPECT_EQ(result.value().out, "");
    EXPECT_EQ(result.value().exit_code, 3);
}

TEST(ExecProcessRunner, ReportsMissingProgramAsExit127) {
    ExecProcessRunner runner;
    const auto result = runner.run({"/definitely/not/a/program"});

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().exit_code, 127);
}

/**
 * @brief stderr 写满管道时不能把 stdout 的读卡死。
 *
 * 审计 H4：旧实现先 `read_all(stdout)` 再 `read_all(stderr)`。子进程把
 * 200 KB 写到 stderr 就会阻塞在管道上（Linux 默认 64 KB），父进程同时在等
 * stdout 的 EOF —— 双方互等，`run()` 永不返回。这条用例在旧实现上不是失败
 * 而是**挂死**，负向对照用 `timeout` 观察（退出码 124）。
 */
TEST(ExecProcessRunner, LargeStderrDoesNotDeadlockQuietStdout) {
    const auto result = run_script("head -c 200000 /dev/zero 1>&2; printf done");

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().out, "done");
    EXPECT_EQ(result.value().err.size(), 200000U);
    EXPECT_EQ(result.value().exit_code, 0);
}

/// @brief 反向：stdout 写满管道时同样不能被 stderr 挡住。
TEST(ExecProcessRunner, LargeStdoutDoesNotDeadlockQuietStderr) {
    const auto result = run_script("head -c 200000 /dev/zero; printf tail 1>&2");

    ASSERT_TRUE(result.ok()) << result.error().message;
    EXPECT_EQ(result.value().out.size(), 200000U);
    EXPECT_EQ(result.value().err, "tail");
}

} // namespace
} // namespace foray::platform::net
