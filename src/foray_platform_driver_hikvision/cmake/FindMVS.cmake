# 定位 Hikvision MVS（机器视觉相机 SDK）。
#
# 用法：
#   find_package(MVS REQUIRED)
#   target_link_libraries(app PRIVATE MVS::MvCameraControl)
#
# 定位顺序：
#   1. 若 `MVS::MvCameraControl` 已存在，直接复用；
#   2. 先按 CONFIG 模式找 `MVSConfig.cmake`（`ZQU-Foray/mvs-sdk` 私有仓或官方安装
#      目录都可能提供），`MVS_ROOT` / `CMAKE_PREFIX_PATH` 指向它即可；
#   3. 再按本模块的 find_path/find_library 探测未提供 config 的安装树
#      （官方 `/opt/MVS` 就属于这种）。
#
# 探测提示变量：
#   FORAY_MVS_ROOT      本仓约定的显式根目录（优先）
#   MVS_ROOT            MVS 通用根目录变量
#   MVCAM_SDK_PATH      MVS 客户端安装时写入的环境变量
#
# 产出：
#   MVS_FOUND / MVS_INCLUDE_DIRS / MVS_LIBRARIES / MVS_VERSION
#   MVS::MvCameraControl  IMPORTED SHARED 目标

if(TARGET MVS::MvCameraControl)
    set(MVS_FOUND TRUE)
    if(NOT MVS_INCLUDE_DIRS)
        get_target_property(MVS_INCLUDE_DIRS MVS::MvCameraControl
            INTERFACE_INCLUDE_DIRECTORIES)
    endif()
    if(NOT MVS_LIBRARIES)
        get_target_property(MVS_LIBRARIES MVS::MvCameraControl
            IMPORTED_LOCATION)
    endif()
    return()
endif()

# 第 2 步：有 CMake package config 的安装树优先，避免重复推断架构目录。
find_package(MVS CONFIG QUIET)
if(MVS_FOUND)
    return()
endif()

# 第 3 步：手动探测。PATH_SUFFIXES 同时覆盖官方布局（lib/64、lib/32）
# 与本仓布局（lib/amd64）。
find_path(MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    HINTS
        ${FORAY_MVS_ROOT}
        ${MVS_ROOT}
        ENV MVCAM_SDK_PATH
        ENV MVS_ROOT
    PATHS
        /opt/MVS
    PATH_SUFFIXES
        include
        include/hikrobot
)

find_library(MVS_MVCC_LIBRARY
    NAMES MvCameraControl
    HINTS
        ${FORAY_MVS_ROOT}
        ${MVS_ROOT}
        ENV MVCAM_SDK_PATH
        ENV MVS_ROOT
    PATHS
        /opt/MVS
    PATH_SUFFIXES
        lib
        lib/64
        lib/x64
        lib/amd64
        lib/aarch64
        lib/arm64
        lib/32
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MVS
    REQUIRED_VARS MVS_MVCC_LIBRARY MVS_INCLUDE_DIR
    VERSION_VAR MVS_VERSION
)

if(MVS_FOUND)
    # MVS 只发布共享库；静态归档会额外传播 libstdc++/pthread，本驱动不接受。
    if(NOT MVS_MVCC_LIBRARY MATCHES "\\.so(\\..*)?$")
        message(FATAL_ERROR
            "找到的是非共享库 MVS: ${MVS_MVCC_LIBRARY}。"
            "请安装 libMvCameraControl.so。")
    endif()

    if(NOT MVS_VERSION)
        # 版本号写在 MVS 安装目录的 MVS_VERSION 或 ReleaseNote 里，两者都不保证
        # 存在；这里不猜，交给运行时 MV_CC_GetSDKVersion。
        set(MVS_VERSION "unknown")
    endif()

    if(NOT TARGET MVS::MvCameraControl)
        add_library(MVS::MvCameraControl SHARED IMPORTED)
        set_target_properties(MVS::MvCameraControl PROPERTIES
            IMPORTED_LOCATION "${MVS_MVCC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${MVS_INCLUDE_DIR}"
        )
    endif()

    set(MVS_INCLUDE_DIRS "${MVS_INCLUDE_DIR}")
    set(MVS_LIBRARIES "${MVS_MVCC_LIBRARY}")
endif()

mark_as_advanced(MVS_INCLUDE_DIR MVS_MVCC_LIBRARY)
