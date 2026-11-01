#include "foray/platform/net/process.hpp"

#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace foray::platform::net {
namespace {

void close_fd(int& fd) {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

/// @brief 同时把 stdout / stderr 读到 EOF。
///
/// 顺序读会死锁：子进程写满其中一个管道（Linux 默认 64 KB）就会阻塞，
/// 而父进程正卡在另一个管道的 `read()` 上等 EOF（审计 H4）。`nmcli` /
/// `foray_net_helper` 这类工具在失败时会把大段日志写到 stderr，正好踩中。
void read_both(int out_fd, int err_fd, std::string& out, std::string& err) {
    pollfd fds[2] = {{out_fd, POLLIN, 0}, {err_fd, POLLIN, 0}};
    std::string* sinks[2] = {&out, &err};
    char buffer[4096];
    while (fds[0].fd >= 0 || fds[1].fd >= 0) {
        const int ready = ::poll(fds, 2, -1);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        for (int index = 0; index < 2; ++index) {
            if (fds[index].fd < 0) {
                continue;
            }
            if (fds[index].revents == 0) {
                continue;
            }
            const ssize_t count = ::read(fds[index].fd, buffer, sizeof(buffer));
            if (count > 0) {
                sinks[index]->append(buffer, static_cast<std::size_t>(count));
                continue;
            }
            if (count < 0 && errno == EINTR) {
                continue;
            }
            // EOF 或读错误：这个管道不会再有数据，停止监听它。
            fds[index].fd = -1;
        }
    }
}

} // namespace

Result<ProcessOutput> ExecProcessRunner::run(const std::vector<std::string>& argv) {
    if (argv.empty()) {
        return Error{ErrorCode::kInvalidArgument, "执行命令失败：argv 为空"};
    }

    int out_pipe[2] = {-1, -1};
    int err_pipe[2] = {-1, -1};
    if (::pipe(out_pipe) != 0 || ::pipe(err_pipe) != 0) {
        close_fd(out_pipe[0]);
        close_fd(out_pipe[1]);
        close_fd(err_pipe[0]);
        close_fd(err_pipe[1]);
        return Error{ErrorCode::kInternal, std::string("创建管道失败: ") + std::strerror(errno)};
    }

    std::vector<char*> args;
    args.reserve(argv.size() + 1);
    for (const auto& arg : argv) {
        args.push_back(const_cast<char*>(arg.c_str()));
    }
    args.push_back(nullptr);

    const pid_t pid = ::fork();
    if (pid < 0) {
        close_fd(out_pipe[0]);
        close_fd(out_pipe[1]);
        close_fd(err_pipe[0]);
        close_fd(err_pipe[1]);
        return Error{ErrorCode::kInternal, std::string("fork 失败: ") + std::strerror(errno)};
    }
    if (pid == 0) {
        ::dup2(out_pipe[1], STDOUT_FILENO);
        ::dup2(err_pipe[1], STDERR_FILENO);
        close_fd(out_pipe[0]);
        close_fd(out_pipe[1]);
        close_fd(err_pipe[0]);
        close_fd(err_pipe[1]);
        ::execvp(args[0], args.data());
        ::_exit(127);
    }

    close_fd(out_pipe[1]);
    close_fd(err_pipe[1]);
    ProcessOutput output;
    read_both(out_pipe[0], err_pipe[0], output.out, output.err);
    close_fd(out_pipe[0]);
    close_fd(err_pipe[0]);

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    output.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return output;
}

} // namespace foray::platform::net
