#include "foray/platform/net/process.hpp"

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

std::string read_all(int fd) {
    std::string data;
    char buffer[4096];
    for (;;) {
        const ssize_t count = ::read(fd, buffer, sizeof(buffer));
        if (count > 0) {
            data.append(buffer, static_cast<std::size_t>(count));
        } else if (count == 0) {
            break;
        } else if (errno != EINTR) {
            break;
        }
    }
    return data;
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
    output.out = read_all(out_pipe[0]);
    output.err = read_all(err_pipe[0]);
    close_fd(out_pipe[0]);
    close_fd(err_pipe[0]);

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    output.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return output;
}

} // namespace foray::platform::net
