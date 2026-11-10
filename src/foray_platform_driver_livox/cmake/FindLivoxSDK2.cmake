# 定位 Livox SDK2。
#
# 用法：
#   find_package(LivoxSDK2 REQUIRED)
#   target_link_libraries(app PRIVATE LivoxSDK2::shared)
#
# 可通过 `LIVOX_SDK2_ROOT`（CMake 变量或环境变量）指定安装前缀。
# SDK2 没有官方 CMake package config，因此本模块只暴露共享库 imported target。

find_path(LivoxSDK2_INCLUDE_DIR
    NAMES livox_lidar_api.h
    HINTS
        ${LIVOX_SDK2_ROOT}
        ENV LIVOX_SDK2_ROOT
    PATH_SUFFIXES include
)

find_library(LivoxSDK2_LIBRARY
    NAMES livox_lidar_sdk_shared
    HINTS
        ${LIVOX_SDK2_ROOT}
        ENV LIVOX_SDK2_ROOT
    PATH_SUFFIXES
        lib
        lib64
        lib/${CMAKE_LIBRARY_ARCHITECTURE}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LivoxSDK2
    REQUIRED_VARS LivoxSDK2_LIBRARY LivoxSDK2_INCLUDE_DIR
)

if(LivoxSDK2_FOUND)
    # 驱动按共享库设计：静态 SDK 会额外传播 pthread/运行库，当前没有收益。
    if(NOT LivoxSDK2_LIBRARY MATCHES "\\.so(\\..*)?$")
        message(FATAL_ERROR
            "找到的是非共享库 Livox SDK2: ${LivoxSDK2_LIBRARY}。"
            "请安装 liblivox_lidar_sdk_shared.so。")
    endif()

    if(NOT TARGET LivoxSDK2::shared)
        add_library(LivoxSDK2::shared SHARED IMPORTED)
        set_target_properties(LivoxSDK2::shared PROPERTIES
            IMPORTED_LOCATION "${LivoxSDK2_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LivoxSDK2_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(LivoxSDK2_INCLUDE_DIR LivoxSDK2_LIBRARY)
