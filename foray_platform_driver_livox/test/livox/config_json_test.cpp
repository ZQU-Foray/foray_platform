#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "livox/config_json.hpp"

namespace foray::platform::driver::livox::detail {
namespace {

std::filesystem::path make_temp_directory() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        ("foray_livox_config_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(directory);
    return directory;
}

TEST(ConfigJsonTest, ValidatesIpv4) {
    EXPECT_TRUE(is_valid_ipv4("127.0.0.1"));
    EXPECT_TRUE(is_valid_ipv4("192.168.1.51"));
    EXPECT_FALSE(is_valid_ipv4("256.1.1.1"));
    EXPECT_FALSE(is_valid_ipv4("1.2.3"));
    EXPECT_FALSE(is_valid_ipv4(""));
    EXPECT_TRUE(is_local_ipv4("127.0.0.1"));
}

TEST(ConfigJsonTest, BuildsPortsAndHostIp) {
    const std::string json = build_config_json("192.168.1.51");
    EXPECT_NE(json.find("\"MID360\""), std::string::npos);
    EXPECT_NE(json.find("\"host_ip\": \"192.168.1.51\""), std::string::npos);
    EXPECT_NE(json.find("\"point_data_port\": 56300"), std::string::npos);
    EXPECT_NE(json.find("\"point_data_port\": 56301"), std::string::npos);
    EXPECT_NE(json.find("\"imu_data_port\": 56401"), std::string::npos);
    EXPECT_NE(json.find("\"host_net_info\""), std::string::npos);
}

TEST(ConfigJsonTest, WritesPrivateConfigFile) {
    const std::filesystem::path directory = make_temp_directory();
    const std::filesystem::path path = directory / "livox.json";

    const Result<void> written = write_config_json(path.string(), "127.0.0.1");
    ASSERT_TRUE(written.ok()) << written.error().message;

    std::ifstream stream(path);
    ASSERT_TRUE(stream.good());
    std::ostringstream content;
    content << stream.rdbuf();
    EXPECT_NE(content.str().find("\"127.0.0.1\""), std::string::npos);

    const auto permissions = std::filesystem::status(path).permissions();
    EXPECT_EQ(permissions & std::filesystem::perms::owner_read, std::filesystem::perms::owner_read);
    EXPECT_EQ(permissions & std::filesystem::perms::owner_write,
              std::filesystem::perms::owner_write);
    EXPECT_EQ(permissions & std::filesystem::perms::group_all, std::filesystem::perms::none);
    EXPECT_EQ(permissions & std::filesystem::perms::others_all, std::filesystem::perms::none);

    std::filesystem::remove_all(directory);
}

TEST(ConfigJsonTest, RejectsInvalidHostIp) {
    const std::filesystem::path directory = make_temp_directory();
    const std::filesystem::path path = directory / "livox.json";
    const Result<void> written = write_config_json(path.string(), "not-an-ip");
    EXPECT_FALSE(written.ok());
    EXPECT_EQ(written.error().code, ErrorCode::kInvalidArgument);
    std::filesystem::remove_all(directory);
}

TEST(ConfigJsonTest, DefaultPathSanitizesIp) {
    const std::string path = default_config_path("/tmp/foray_runtime", "192.168.1.51");
    EXPECT_NE(path.find("foray_runtime"), std::string::npos);
    EXPECT_NE(path.find("livox_config_192_168_1_51.json"), std::string::npos);
}

} // namespace
} // namespace foray::platform::driver::livox::detail
