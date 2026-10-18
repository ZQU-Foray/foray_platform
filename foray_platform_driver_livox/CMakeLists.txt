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

if(FORAY_BUILD_DRIVER_LIVOX)
    set(FORAY_DRIVER_LIVOX_HAS_SDK_VALUE 1)
else()
    set(FORAY_DRIVER_LIVOX_HAS_SDK_VALUE 0)
endif()

set(FORAY_LIVOX_SDK_PROVIDER "SYSTEM" CACHE STRING
    "Livox SDK2 provider: SYSTEM (system paths/CMAKE_PREFIX_PATH) or PREBUILT (LIVOX_SDK2_ROOT)")
set_property(CACHE FORAY_LIVOX_SDK_PROVIDER PROPERTY STRINGS SYSTEM PREBUILT)

find_package(foray_platform REQUIRED)
find_package(Threads REQUIRED)

set(FORAY_LIVOX_CORE_SOURCES
    src/livox/lidar.cpp
    src/livox/livox_runtime.cpp
    src/livox/packet_parser.cpp
    src/livox/frame_assembler.cpp
    src/livox/config_json.cpp
)

set(FORAY_LIVOX_BACKEND_SOURCES
    src/livox/lidar_public_factory.cpp
)

if(FORAY_BUILD_DRIVER_LIVOX)
    if(NOT FORAY_LIVOX_SDK_PROVIDER STREQUAL "SYSTEM"
       AND NOT FORAY_LIVOX_SDK_PROVIDER STREQUAL "PREBUILT")
        message(FATAL_ERROR
            "FORAY_LIVOX_SDK_PROVIDER 只能是 SYSTEM 或 PREBUILT，"
            "当前为 ${FORAY_LIVOX_SDK_PROVIDER}")
    endif()

    if(FORAY_LIVOX_SDK_PROVIDER STREQUAL "PREBUILT" AND NOT LIVOX_SDK2_ROOT)
        message(FATAL_ERROR
            "FORAY_LIVOX_SDK_PROVIDER=PREBUILT 时必须设置 LIVOX_SDK2_ROOT。")
    endif()

    list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package(LivoxSDK2 REQUIRED)

    if(FORAY_LIVOX_SDK_PROVIDER STREQUAL "PREBUILT")
        # 避免误用系统里的另一份 SDK：头文件与库都必须位于指定前缀下。
        get_filename_component(_foray_livox_root "${LIVOX_SDK2_ROOT}" REALPATH)
        get_filename_component(_foray_livox_include "${LivoxSDK2_INCLUDE_DIR}" REALPATH)
        get_filename_component(_foray_livox_library "${LivoxSDK2_LIBRARY}" REALPATH)
        string(FIND "${_foray_livox_include}" "${_foray_livox_root}/" _include_position)
        string(FIND "${_foray_livox_library}" "${_foray_livox_root}/" _library_position)
        if(NOT _include_position EQUAL 0 OR NOT _library_position EQUAL 0)
            message(FATAL_ERROR
                "PREBUILT 模式要求 SDK 位于 LIVOX_SDK2_ROOT=${LIVOX_SDK2_ROOT} 之下，"
                "实际 include=${LivoxSDK2_INCLUDE_DIR}，library=${LivoxSDK2_LIBRARY}")
        endif()
    endif()

    list(APPEND FORAY_LIVOX_BACKEND_SOURCES src/livox/livox_api_real.cpp)
    message(STATUS "Livox SDK2 (${FORAY_LIVOX_SDK_PROVIDER}): ${LivoxSDK2_LIBRARY}")
    message(STATUS "Livox SDK2 include: ${LivoxSDK2_INCLUDE_DIR}")
else()
    list(APPEND FORAY_LIVOX_BACKEND_SOURCES src/livox/livox_api_stub.cpp)
    message(STATUS "Livox SDK2 未启用；驱动使用 stub（open 返回 kSdkUnavailable）")
endif()

# 核心对象：不引用 make_livox_api()，因此 mock 测试可在任何开关下不链接 SDK。
add_library(foray_livox_core_objects OBJECT ${FORAY_LIVOX_CORE_SOURCES})
target_include_directories(foray_livox_core_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(foray_livox_core_objects PUBLIC foray_platform::hal)

# 后端对象：真实 SDK 或 stub，二选一。
add_library(foray_livox_backend_objects OBJECT ${FORAY_LIVOX_BACKEND_SOURCES})
target_include_directories(foray_livox_backend_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(foray_livox_backend_objects PUBLIC foray_platform::hal)
if(FORAY_BUILD_DRIVER_LIVOX)
    target_link_libraries(foray_livox_backend_objects PRIVATE LivoxSDK2::shared)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_livox_core_objects PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(foray_livox_backend_objects PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(foray_platform_driver_livox
    $<TARGET_OBJECTS:foray_livox_core_objects>
    $<TARGET_OBJECTS:foray_livox_backend_objects>
)
add_library(foray_platform::driver_livox ALIAS foray_platform_driver_livox)

set_target_properties(foray_platform_driver_livox PROPERTIES
    EXPORT_NAME driver_livox
    INSTALL_RPATH_USE_LINK_PATH FALSE
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
    target_link_libraries(foray_platform_driver_livox PRIVATE LivoxSDK2::shared)
endif()

# 描述“已安装产物”的能力：1=真实 SDK 后端，0=stub。
target_compile_definitions(foray_platform_driver_livox
    PUBLIC FORAY_DRIVER_LIVOX_HAS_SDK=${FORAY_DRIVER_LIVOX_HAS_SDK_VALUE})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_platform_driver_livox PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(FORAY_PLATFORM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # 协议/生命周期测试：只链 core 对象 + mock，永远不依赖 SDK。
    add_executable(foray_platform_driver_livox_tests
        test/livox/packet_parser_test.cpp
        test/livox/config_json_test.cpp
        test/livox/frame_assembler_test.cpp
        test/livox/lidar_test.cpp
    )
    target_include_directories(foray_platform_driver_livox_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(foray_platform_driver_livox_tests
        PRIVATE
            foray_livox_core_objects
            foray_platform::hal
            GTest::gtest_main
            Threads::Threads)

    # 后端能力测试：OFF 断言 kSdkUnavailable，ON 只验证构造，不打开设备。
    add_executable(foray_platform_driver_livox_backend_tests
        test/livox/backend_test.cpp
    )
    target_include_directories(foray_platform_driver_livox_backend_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(foray_platform_driver_livox_backend_tests
        PRIVATE
            foray_platform_driver_livox
            GTest::gtest_main
            Threads::Threads)

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_platform_driver_livox_tests
            PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(foray_platform_driver_livox_backend_tests
            PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(foray_platform_driver_livox_tests)
    gtest_discover_tests(foray_platform_driver_livox_backend_tests)
endif()

set(FORAY_DRIVER_LIVOX_NEEDS_SDK_AT_LINK 0)
if(FORAY_BUILD_DRIVER_LIVOX)
    get_target_property(_foray_livox_library_type foray_platform_driver_livox TYPE)
    if(_foray_livox_library_type STREQUAL "STATIC_LIBRARY")
        set(FORAY_DRIVER_LIVOX_NEEDS_SDK_AT_LINK 1)
    endif()
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

install(FILES cmake/FindLivoxSDK2.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_livox)

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
