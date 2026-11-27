# 通信と機器単体の制限。URDF・床・衝突判定に依存しない。
add_library(topoarm_driver OBJECT dynamixel_handler.cpp
  ${DXL}/download/port_handler_fix.cpp ${TOPOARM_SERIAL_SOURCE}
  ${DXL}/src/dynamixel_communicator_main.cpp ${DXL}/src/dynamixel_communicator_retry.cpp
  ${DXL}/src/dynamixel_parameters.cpp)
target_include_directories(topoarm_driver PRIVATE ../include ${DXL}/src ${DXL}/download)
target_compile_features(topoarm_driver PRIVATE cxx_std_17)
set_target_properties(topoarm_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
