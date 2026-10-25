cmake_minimum_required(VERSION 3.8)
project(foray_platform)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/usb_cdc_port.cpp
    src/robolink_crc.cpp
    src/robolink_frame.cpp
    src/robolink_frame_parser.cpp
    src/robolink_codec.cpp
    src/robolink_byte_channel.cpp
    src/robolink_host_link.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_usb_cdc_port test/test_usb_cdc_port.cpp)
    target_link_libraries(test_usb_cdc_port ${PROJECT_NAME} util)
    ament_add_gtest(test_robolink_crc test/test_robolink_crc.cpp)
    target_link_libraries(test_robolink_crc ${PROJECT_NAME})
    ament_add_gtest(test_robolink_frame test/test_robolink_frame.cpp)
    target_link_libraries(test_robolink_frame ${PROJECT_NAME})
    ament_add_gtest(test_robolink_frame_parser test/test_robolink_frame_parser.cpp)
    target_link_libraries(test_robolink_frame_parser ${PROJECT_NAME})
    ament_add_gtest(test_robolink_codec test/test_robolink_codec.cpp)
    target_link_libraries(test_robolink_codec ${PROJECT_NAME})
    ament_add_gtest(test_robolink_byte_channel test/test_robolink_byte_channel.cpp)
    target_link_libraries(test_robolink_byte_channel ${PROJECT_NAME} util)
    ament_add_gtest(test_robolink_host_link test/test_robolink_host_link.cpp)
    target_link_libraries(test_robolink_host_link ${PROJECT_NAME} util)
endif()

ament_package()
