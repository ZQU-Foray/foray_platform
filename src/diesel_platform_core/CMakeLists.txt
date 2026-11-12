cmake_minimum_required(VERSION 3.16)
project(diesel_platform_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(DIESEL_PLATFORM_BUILD_TESTS "Build diesel_platform unit tests" ON)

find_package(yaml-cpp REQUIRED)

add_library(diesel_platform_core
    src/time/clock.cpp
    src/result/error.cpp
    src/param/value.cpp
    src/param/result.cpp
    src/param/yaml_utils.cpp
    src/param/validation.cpp
    src/param/schema.cpp
    src/param/snapshot.cpp
    src/param/store.cpp
    src/param/source.cpp
    src/param/yaml_source.cpp
    src/param/manager.cpp
    src/config/resolver.cpp
    src/config/exporter.cpp
    src/config/tool.cpp
)
add_library(diesel_platform::core ALIAS diesel_platform_core)

# 安装后的导入名保持 diesel_platform::core（命名空间不随包名变化）。
set_target_properties(diesel_platform_core PROPERTIES EXPORT_NAME core)

add_library(diesel_platform_hal
    src/hal/config_binder.cpp
)
add_library(diesel_platform::hal ALIAS diesel_platform_hal)
set_target_properties(diesel_platform_hal PROPERTIES EXPORT_NAME hal)
target_include_directories(diesel_platform_hal
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(diesel_platform_hal PUBLIC diesel_platform_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diesel_platform_hal PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(diesel_platform_net
    src/net/probe.cpp
    src/net/process.cpp
    src/net/nm_backend.cpp
    src/net/netlink_backend.cpp
    src/net/setup.cpp
    src/net/bootstrap.cpp
)
add_library(diesel_platform::net ALIAS diesel_platform_net)
set_target_properties(diesel_platform_net PROPERTIES EXPORT_NAME net)
target_include_directories(diesel_platform_net
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(diesel_platform_net PUBLIC diesel_platform_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diesel_platform_net PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(diesel_platform_link
    src/link/usb_cdc_port.cpp
    src/link/robolink_crc.cpp
    src/link/robolink_frame.cpp
    src/link/robolink_frame_parser.cpp
    src/link/robolink_codec.cpp
    src/link/robolink_byte_channel.cpp
    src/link/robolink_host_link.cpp
)
add_library(diesel_platform::link ALIAS diesel_platform_link)
set_target_properties(diesel_platform_link PROPERTIES EXPORT_NAME link)
target_include_directories(diesel_platform_link
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(diesel_platform_link PUBLIC diesel_platform_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diesel_platform_link PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(diesel_platform_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(TARGET yaml-cpp::yaml-cpp)
    set(DIESEL_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(DIESEL_YAML_CPP_TARGET yaml-cpp)
endif()
target_link_libraries(diesel_platform_core PUBLIC ${DIESEL_YAML_CPP_TARGET})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diesel_platform_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(DIESEL_PLATFORM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(diesel_platform_tests
        test/time/clock_test.cpp
        test/result/result_test.cpp
        test/container/spsc_queue_test.cpp
        test/param/value_test.cpp
        test/param/schema_test.cpp
        test/param/store_test.cpp
        test/param/yaml_source_test.cpp
        test/param/manager_test.cpp
        test/config/resolver_test.cpp
        test/config/exporter_test.cpp
        test/hal/types_test.cpp
        test/hal/frame_pool_test.cpp
        test/hal/config_binder_test.cpp
        test/hal/fake_sensor_test.cpp
        test/net/net_types_test.cpp
        test/net/probe_test.cpp
        test/net/setup_test.cpp
        test/net/nm_backend_test.cpp
        test/net/bootstrap_test.cpp
        test/link/usb_cdc_port_test.cpp
        test/link/robolink_crc_test.cpp
        test/link/robolink_frame_test.cpp
        test/link/robolink_frame_parser_test.cpp
        test/link/robolink_codec_test.cpp
        test/link/robolink_byte_channel_test.cpp
        test/link/robolink_host_link_test.cpp
    )
    target_link_libraries(diesel_platform_tests
        PRIVATE diesel_platform_link diesel_platform_net diesel_platform_hal
                diesel_platform_core GTest::gtest_main
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(diesel_platform_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(diesel_platform_tests)
endif()

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# 特权网络 helper：只能由 pkexec/polkit 以 root 启动，安装到 libexec 避免被
# 普通用户直接调用；实际参数白名单在工具内校验。
add_executable(diesel-net-helper apps/diesel_net_helper.cpp)
target_link_libraries(diesel-net-helper PRIVATE diesel_platform_net)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(diesel-net-helper PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS diesel-net-helper RUNTIME DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/${PROJECT_NAME})

# polkit：把 helper 注册成 org.diesel.netconfig.bootstrap，pkexec 只放行该程序。
set(DIESEL_HELPER_PATH
    "${CMAKE_INSTALL_FULL_LIBEXECDIR}/${PROJECT_NAME}/diesel-net-helper")
configure_file(data/polkit/org.diesel.netconfig.policy.in
    ${CMAKE_CURRENT_BINARY_DIR}/org.diesel.netconfig.policy
    @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/org.diesel.netconfig.policy
    DESTINATION ${CMAKE_INSTALL_DATADIR}/polkit-1/actions)
# 可选免密规则：装到 rules.d 后 diesel 组成员不再弹授权。
install(FILES data/polkit/60-diesel-netconfig.rules
    DESTINATION ${CMAKE_INSTALL_DATADIR}/polkit-1/rules.d)

install(TARGETS diesel_platform_core diesel_platform_hal diesel_platform_net diesel_platform_link
    EXPORT diesel_platform_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT diesel_platform_coreTargets
    FILE diesel_platform_coreTargets.cmake
    NAMESPACE diesel_platform::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/diesel_platform_core
)

configure_package_config_file(
    cmake/diesel_platform_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/diesel_platform_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/diesel_platform_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/diesel_platform_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/diesel_platform_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/diesel_platform_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/diesel_platform_core
)
