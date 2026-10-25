cmake_minimum_required(VERSION 3.16)
project(foray_platform_driver_hikvision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(FORAY_BUILD_DRIVER_HIKVISION "Build the driver against Hikvision MVS" OFF)
option(FORAY_PLATFORM_BUILD_TESTS "Build driver unit tests" ON)
option(FORAY_HIKVISION_BUILD_TOOLS "Build Hikvision diagnostic tools (requires driver SDK ON)" OFF)

if(FORAY_BUILD_DRIVER_HIKVISION)
    set(FORAY_DRIVER_HIKVISION_HAS_SDK_VALUE 1)
else()
    set(FORAY_DRIVER_HIKVISION_HAS_SDK_VALUE 0)
endif()

set(FORAY_MVS_PROVIDER "SYSTEM" CACHE STRING
    "MVS provider: SYSTEM (system paths/CMAKE_PREFIX_PATH, e.g. /opt/MVS) or PREBUILT (FORAY_MVS_ROOT)")
set_property(CACHE FORAY_MVS_PROVIDER PROPERTY STRINGS SYSTEM PREBUILT)

find_package(foray_platform REQUIRED)
find_package(Threads REQUIRED)

set(FORAY_HIKVISION_CORE_SOURCES
    src/hikvision/camera.cpp
    src/hikvision/camera_bootstrap.cpp
    src/hikvision/camera_factory.cpp
    src/hikvision/mvs_error.cpp
    src/hikvision/mvs_nodes.cpp
    src/hikvision/timestamp.cpp
)

set(FORAY_HIKVISION_BACKEND_SOURCES
    src/hikvision/camera_public_factory.cpp
)

if(FORAY_BUILD_DRIVER_HIKVISION)
    if(NOT FORAY_MVS_PROVIDER STREQUAL "SYSTEM"
       AND NOT FORAY_MVS_PROVIDER STREQUAL "PREBUILT")
        message(FATAL_ERROR
            "FORAY_MVS_PROVIDER 只能是 SYSTEM 或 PREBUILT，"
            "当前为 ${FORAY_MVS_PROVIDER}")
    endif()

    if(FORAY_MVS_PROVIDER STREQUAL "PREBUILT" AND NOT FORAY_MVS_ROOT)
        message(FATAL_ERROR
            "FORAY_MVS_PROVIDER=PREBUILT 时必须设置 FORAY_MVS_ROOT。")
    endif()

    list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package(MVS REQUIRED)

    get_target_property(_foray_mvs_include MVS::MvCameraControl
        INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(_foray_mvs_library MVS::MvCameraControl
        IMPORTED_LOCATION)

    if(FORAY_MVS_PROVIDER STREQUAL "PREBUILT")
        # 避免误用系统里的另一份 SDK：头文件与库都必须位于指定前缀下。
        get_filename_component(_foray_mvs_root "${FORAY_MVS_ROOT}" REALPATH)
        get_filename_component(_foray_mvs_include_real "${_foray_mvs_include}" REALPATH)
        get_filename_component(_foray_mvs_library_real "${_foray_mvs_library}" REALPATH)
        string(FIND "${_foray_mvs_include_real}" "${_foray_mvs_root}/" _include_position)
        string(FIND "${_foray_mvs_library_real}" "${_foray_mvs_root}/" _library_position)
        if(NOT _include_position EQUAL 0 OR NOT _library_position EQUAL 0)
            message(FATAL_ERROR
                "PREBUILT 模式要求 SDK 位于 FORAY_MVS_ROOT=${FORAY_MVS_ROOT} 之下，"
                "实际 include=${_foray_mvs_include}，library=${_foray_mvs_library}")
        endif()
    endif()

    list(APPEND FORAY_HIKVISION_BACKEND_SOURCES src/hikvision/mvs_api_real.cpp)
    message(STATUS "MVS (${FORAY_MVS_PROVIDER}): ${_foray_mvs_library}")
    message(STATUS "MVS include: ${_foray_mvs_include}")
else()
    list(APPEND FORAY_HIKVISION_BACKEND_SOURCES src/hikvision/mvs_api_stub.cpp)
    message(STATUS "MVS 未启用；驱动使用 stub（open 返回 kSdkUnavailable）")
endif()

# 核心对象：不引用 make_mvs_api()，因此 mock 测试可在任何开关下不链接 SDK。
add_library(foray_hikvision_core_objects OBJECT ${FORAY_HIKVISION_CORE_SOURCES})
target_include_directories(foray_hikvision_core_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(foray_hikvision_core_objects PUBLIC foray_platform::hal
                                                    foray_platform::net)

# 后端对象：真实 SDK 或 stub，二选一。
add_library(foray_hikvision_backend_objects OBJECT ${FORAY_HIKVISION_BACKEND_SOURCES})
target_include_directories(foray_hikvision_backend_objects
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(foray_hikvision_backend_objects PUBLIC foray_platform::hal)
if(FORAY_BUILD_DRIVER_HIKVISION)
    target_link_libraries(foray_hikvision_backend_objects PRIVATE MVS::MvCameraControl)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_hikvision_core_objects PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(foray_hikvision_backend_objects PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(foray_platform_driver_hikvision
    $<TARGET_OBJECTS:foray_hikvision_core_objects>
    $<TARGET_OBJECTS:foray_hikvision_backend_objects>
)
add_library(foray_platform::driver_hikvision ALIAS foray_platform_driver_hikvision)

set_target_properties(foray_platform_driver_hikvision PROPERTIES
    EXPORT_NAME driver_hikvision
    INSTALL_RPATH_USE_LINK_PATH FALSE
)

target_include_directories(foray_platform_driver_hikvision
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(foray_platform_driver_hikvision
    PUBLIC foray_platform::hal foray_platform::net
    PRIVATE Threads::Threads
)
if(FORAY_BUILD_DRIVER_HIKVISION)
    target_link_libraries(foray_platform_driver_hikvision PRIVATE MVS::MvCameraControl)
endif()

# 描述“已安装产物”的能力：1=真实 SDK 后端，0=stub。
target_compile_definitions(foray_platform_driver_hikvision
    PUBLIC FORAY_DRIVER_HIKVISION_HAS_SDK=${FORAY_DRIVER_HIKVISION_HAS_SDK_VALUE})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_platform_driver_hikvision PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(FORAY_PLATFORM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # 生命周期/配置映射/时间换算：只链 core 对象 + mock，永远不依赖 SDK。
    add_executable(foray_platform_driver_hikvision_tests
        test/hikvision/camera_test.cpp
        test/hikvision/camera_bootstrap_test.cpp
        test/hikvision/config_mapping_test.cpp
        test/hikvision/timestamp_test.cpp
        test/hikvision/mvs_error_test.cpp
    )
    target_include_directories(foray_platform_driver_hikvision_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(foray_platform_driver_hikvision_tests
        PRIVATE
            foray_hikvision_core_objects
            foray_platform::hal
            foray_platform::net
            GTest::gtest_main
            Threads::Threads)

    # 后端能力测试：OFF 断言 kSdkUnavailable，ON 只验证构造，不打开设备。
    add_executable(foray_platform_driver_hikvision_backend_tests
        test/hikvision/backend_test.cpp
    )
    target_include_directories(foray_platform_driver_hikvision_backend_tests
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(foray_platform_driver_hikvision_backend_tests
        PRIVATE
            foray_platform_driver_hikvision
            GTest::gtest_main
            Threads::Threads)

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_platform_driver_hikvision_tests
            PRIVATE -Wall -Wextra -Wpedantic)
        target_compile_options(foray_platform_driver_hikvision_backend_tests
            PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(foray_platform_driver_hikvision_tests)
    gtest_discover_tests(foray_platform_driver_hikvision_backend_tests)
endif()

if(FORAY_HIKVISION_BUILD_TOOLS)
    if(NOT FORAY_BUILD_DRIVER_HIKVISION)
        message(FATAL_ERROR
            "FORAY_HIKVISION_BUILD_TOOLS=ON 需要同时设置 "
            "FORAY_BUILD_DRIVER_HIKVISION=ON")
    endif()

    add_executable(foray_hikvision_dump tools/camera_dump.cpp tools/viewer.cpp)
    target_link_libraries(foray_hikvision_dump PRIVATE foray_platform_driver_hikvision)

    # --view 的实时预览依赖 SDL2，但它是可选的：找不到就只少了这个功能，
    # 工具其余部分照常可用（viewer.cpp 里走 FORAY_HIKVISION_HAVE_SDL2=0 的退化实现）。
    find_package(SDL2 QUIET)
    if(TARGET SDL2::SDL2)
        target_link_libraries(foray_hikvision_dump PRIVATE SDL2::SDL2)
    elseif(SDL2_FOUND AND SDL2_INCLUDE_DIRS AND SDL2_LIBRARIES)
        # 部分发行版只提供变量，不给 IMPORTED target。
        target_include_directories(foray_hikvision_dump PRIVATE ${SDL2_INCLUDE_DIRS})
        target_link_libraries(foray_hikvision_dump PRIVATE ${SDL2_LIBRARIES})
    endif()

    if(TARGET SDL2::SDL2 OR (SDL2_FOUND AND SDL2_LIBRARIES))
        set(FORAY_HIKVISION_HAVE_SDL2 1)
        message(STATUS "SDL2 已找到：foray_hikvision_dump 支持 --view 实时预览")
    else()
        set(FORAY_HIKVISION_HAVE_SDL2 0)
        message(STATUS "SDL2 未找到：foray_hikvision_dump 不带 --view 实时预览")
    endif()
    target_compile_definitions(foray_hikvision_dump
        PRIVATE FORAY_HIKVISION_HAVE_SDL2=${FORAY_HIKVISION_HAVE_SDL2})

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_hikvision_dump PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    install(TARGETS foray_hikvision_dump RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    # 一键自举：网络编排（可经 pkexec）+ 相机启动 + 取帧健康确认，与 foray_lidar_up 对齐。
    add_executable(foray_hikvision_up tools/camera_up.cpp)
    target_include_directories(foray_hikvision_up
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    # 安装后的 helper 不在 PATH 里，编译期给一个默认绝对路径。
    target_compile_definitions(foray_hikvision_up PRIVATE
        FORAY_NET_HELPER_DEFAULT="${CMAKE_INSTALL_FULL_LIBEXECDIR}/foray_platform/foray-net-helper")
    target_link_libraries(foray_hikvision_up
        PRIVATE foray_platform_driver_hikvision foray_platform::net)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_hikvision_up PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    install(TARGETS foray_hikvision_up RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

set(FORAY_DRIVER_HIKVISION_NEEDS_SDK_AT_LINK 0)
if(FORAY_BUILD_DRIVER_HIKVISION)
    get_target_property(_foray_hikvision_library_type
        foray_platform_driver_hikvision TYPE)
    if(_foray_hikvision_library_type STREQUAL "STATIC_LIBRARY")
        set(FORAY_DRIVER_HIKVISION_NEEDS_SDK_AT_LINK 1)
    endif()
endif()

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(TARGETS foray_platform_driver_hikvision
    EXPORT foray_platform_driver_hikvisionTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT foray_platform_driver_hikvisionTargets
    FILE foray_platform_driver_hikvisionTargets.cmake
    NAMESPACE foray_platform::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_hikvision
)

install(FILES cmake/FindMVS.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_hikvision)

configure_package_config_file(
    cmake/foray_platform_driver_hikvisionConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_hikvisionConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_hikvision
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_hikvisionConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_hikvisionConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_driver_hikvisionConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_driver_hikvision
)
