cmake_minimum_required(VERSION 3.16)
project(foray_platform_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(FORAY_PLATFORM_BUILD_TESTS "Build foray_platform_core unit tests" ON)

find_package(yaml-cpp REQUIRED)

add_library(foray_platform_core
    src/time/clock.cpp
    src/param/value.cpp
    src/param/result.cpp
    src/param/validation.cpp
    src/param/schema.cpp
    src/param/snapshot.cpp
    src/param/store.cpp
    src/param/source.cpp
    src/param/yaml_source.cpp
    src/param/manager.cpp
)
add_library(foray_platform::core ALIAS foray_platform_core)

# 安装后的导入名保持 foray_platform::core。
set_target_properties(foray_platform_core PROPERTIES EXPORT_NAME core)

target_include_directories(foray_platform_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(TARGET yaml-cpp::yaml-cpp)
    set(FORAY_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(FORAY_YAML_CPP_TARGET yaml-cpp)
endif()
target_link_libraries(foray_platform_core PUBLIC ${FORAY_YAML_CPP_TARGET})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(foray_platform_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(FORAY_PLATFORM_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(foray_platform_core_tests
        test/time/clock_test.cpp
        test/param/value_test.cpp
        test/param/schema_test.cpp
        test/param/store_test.cpp
        test/param/yaml_source_test.cpp
        test/param/manager_test.cpp
    )
    target_link_libraries(foray_platform_core_tests
        PRIVATE foray_platform_core GTest::gtest_main
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(foray_platform_core_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    include(GoogleTest)
    gtest_discover_tests(foray_platform_core_tests)
endif()

install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(TARGETS foray_platform_core
    EXPORT foray_platform_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT foray_platform_coreTargets
    FILE foray_platform_coreTargets.cmake
    NAMESPACE foray_platform::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_core
)

configure_package_config_file(
    cmake/foray_platform_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/foray_platform_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/foray_platform_core
)
