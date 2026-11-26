cmake_minimum_required(VERSION 3.22)
project(topoarm_sdk VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
option(TOPOARM_TEST_DRIVER "模擬バスで実機APIを検証する" OFF)
set(DXL ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib_dynamixel)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(TOPOARM_SERIAL_SOURCE ${DXL}/download/port_handler_linux_fix.cpp)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(TOPOARM_SERIAL_SOURCE ${DXL}/download/port_handler_mac.cpp)
else()
  message(FATAL_ERROR "実機SDKはLinuxまたはmacOSでビルドしてください")
endif()
find_package(yaml-cpp CONFIG REQUIRED)
if(APPLE)
  find_program(_topoarm_brew brew)
  if(_topoarm_brew)
    execute_process(COMMAND "${_topoarm_brew}" --prefix eigen@3
      OUTPUT_VARIABLE _topoarm_eigen_hint OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  endif()
endif()
find_package(Eigen3 3.3 CONFIG REQUIRED HINTS ${_topoarm_eigen_hint})
include(cmake/TopoArmSdkDependencies.cmake)
add_subdirectory(driver)
add_subdirectory(safety)
add_library(topoarm_sdk STATIC src/settings.cpp src/driver.cpp src/config.cpp src/driver_session.cpp
  $<TARGET_OBJECTS:topoarm_driver> $<TARGET_OBJECTS:topoarm_safety>)
add_library(topoarm::sdk ALIAS topoarm_sdk)
if(APPLE)
  target_link_libraries(topoarm_sdk PUBLIC "-framework IOKit" "-framework CoreFoundation")
endif()
target_link_libraries(topoarm_sdk PUBLIC yaml-cpp::yaml-cpp tinyxml2::tinyxml2 Eigen3::Eigen)
target_compile_features(topoarm_sdk PUBLIC cxx_std_17)
set_target_properties(topoarm_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME sdk)
target_include_directories(topoarm_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${DXL}/src>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/lib_dynamixel/src>
  PRIVATE ${DXL}/download)
install(TARGETS topoarm_sdk EXPORT topoarm_sdkTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${DXL}/src/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lib_dynamixel/src FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${DXL}/download/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lib_dynamixel/download FILES_MATCHING PATTERN "*.h")
install(FILES ${DXL}/README.md ${DXL}/LICENSE-ROBOTIS
  DESTINATION ${CMAKE_INSTALL_DATADIR}/topoarm_sdk/third_party/lib_dynamixel)
install(EXPORT topoarm_sdkTargets NAMESPACE topoarm:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/topoarm_sdk)
configure_package_config_file(cmake/topoarm_sdkConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/topoarm_sdkConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/topoarm_sdk)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/topoarm_sdkConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/topoarm_sdkConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/topoarm_sdkConfigVersion.cmake
  cmake/TopoArmSdkDependencies.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/topoarm_sdk)
if(TOPOARM_TEST_DRIVER AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../tests/servo_recovery.cpp")
  enable_testing()
  add_executable(test_servo_recovery ../tests/servo_recovery.cpp)
  target_link_libraries(test_servo_recovery PRIVATE topoarm::sdk)
  add_test(NAME servo_recovery COMMAND test_servo_recovery)
endif()
