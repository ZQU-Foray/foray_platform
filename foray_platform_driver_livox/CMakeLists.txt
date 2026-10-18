cmake_minimum_required(VERSION 3.16)
project(foray_platform_driver_livox VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(FORAY_BUILD_DRIVER_LIVOX "Build the driver against Livox SDK2" OFF)
option(FORAY_PLATFORM_BUILD_TESTS "Build driver unit tests" ON)

find_package(foray_platform REQUIRED)
find_package(Threads REQUIRED)

set(FORAY_LIVOX_SOURCES
    src/livox/lidar.cpp
    src/livox/livox_runtime.cpp
    src/livox/packet_parser.cpp
    src/livox/frame_assembler.cpp
    src/livox/config_json.cpp
)

if(FORAY_BUILD_DRIVER_LIVOX)
    find_path(FORAY_LIVOX_SDK2_INCLUDE_DIR
        NAMES livox_lidar_api.h
        HINTS
            ${LIVOX_SDK2_ROOT}/include
            /usr/local/include
            /usr/include
    )
    find_library(FORAY_LIVOX_SDK2_LIBRARY
        NAMES livox_lidar_sdk_shared
        HINTS
            ${LIVOX_SDK2_ROOT}/lib
            /usr/local/lib
            /usr/lib
    )

    if(NOT FORAY_LIVOX_SDK2_INCLUDE_DIR OR NOT FORAY_LIVOX_SDK2_LIBRARY)
        message(FATAL_ERROR
            "FORAY_BUILD_DRIVER_LIVOX=ON 但找不到 Livox SDK2。"
            "请设置 LIVOX_SDK2_ROOT 指向已安装的 SDK2（包含 include/livox_lidar_api.h "
            "与 lib/livox_lidar_sdk_shared.so）。")
    endif()

    list(APPEND FORAY_LIVOX_SOURCES src/livox/livox_api_real.cpp)
    message(STATUS "Livox SDK2: ${FORAY_LIVOX_SDK2_LIBRARY}")
else()
    list(APPEND FORAY_LIVOX_SOURCES src/livox/livox_api_stub.cpp)
    message(STATUS "Livox SDK2 未启用；驱动使用 stub（open 返回 kSdkUnavailable）")
endif()

add_library(foray_platform_driver_livox ${FORAY_LIVOX_SOURCES})
add_library(foray_platform::driver_livox ALIAS foray_platform_driver_livox)
set_target_properties(foray_platform_driver_livox PROPERTIES
    EXPORT_NAME driver_livox
)

target_include_directories(foray_platform_driver_livox
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(foray_platform_driver_livox
    PUBLIC foray_platform::hal
    PRIVATE Threads::Threads
)

if(FORAY_BUILD_DRIVER_LIVOX)
    target_include_directories(foray_platform_driver_livox
        PRIVATE ${FORAY_LIVOX_SDK2_INCLUDE_DIR})
    target_link_libraries(foray_platform_driver_livox
        PRIVATE ${FORAY_LIVOX_SDK2_LIBRARY})
    target_compile_definitions(foray_platform_driver_livox
        PRIVATE FORAY_DRIVER_LIVOX_HAS_SDK=1)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_platform_driver_livox PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(FORAY_PLATFORM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(foray_platform_driver_livox_tests
        test/livox/packet_parser_test.cpp
        test/livox/config_json_test.cpp
        test/livox/frame_assembler_test.cpp
        test/livox/lidar_test.cpp
    )
    target_include_directories(foray_platform_driver_livox_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(foray_platform_driver_livox_tests
        PRIVATE foray_platform_driver_livox GTest::gtest_main Threads::Threads)

    if(FORAY_BUILD_DRIVER_LIVOX)
        target_compile_definitions(foray_platform_driver_livox_tests
            PRIVATE FORAY_DRIVER_LIVOX_HAS_SDK=1)
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_platform_driver_livox_tests
            PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(foray_platform_driver_livox_tests)
endif()

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(TARGETS foray_platform_driver_livox
    EXPORT foray_platform_driver_livoxTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT foray_platform_driver_livoxTargets
    FILE foray_platform_driver_livoxTargets.cmake
    NAMESPACE foray_platform::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_livox
)

configure_package_config_file(
    cmake/foray_platform_driver_livoxConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_livoxConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_livox
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_livoxConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_livoxConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_livoxConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_livox
)
