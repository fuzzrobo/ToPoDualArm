cmake_minimum_required(VERSION 3.22)
project(topoarm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT "$ENV{ROS_DISTRO}" MATCHES "^(jazzy|humble)$")
  message(FATAL_ERROR "ROS 2はJazzyまたはHumbleを使用してください")
endif()
# リポジトリ内では同一ソースをリンク。外部SDKのinstallにも対応する。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/CMakeLists.txt")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../sdk" "${CMAKE_CURRENT_BINARY_DIR}/sdk")
else()
  find_package(topoarm_sdk CONFIG REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED src/topoarm_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
target_link_libraries(${PROJECT_NAME} topoarm::sdk)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface topoarm_hardware.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include include/lib_dynamixel/src)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
