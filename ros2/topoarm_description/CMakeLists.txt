cmake_minimum_required(VERSION 3.5)
project(topoarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

set(COMMON_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/../../config")
install(FILES "${COMMON_CONFIG}/hardware/leader.yaml" "${COMMON_CONFIG}/hardware/follower.yaml"
  DESTINATION share/${PROJECT_NAME}/config/hardware)
install(FILES "${COMMON_CONFIG}/floor.yaml" DESTINATION share/${PROJECT_NAME}/config)


# 正本はルートmodels/。ROS用URIへの変換結果はビルドディレクトリだけに置く。
set(COMMON_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/../../models")
foreach(model robot.urdf pan_tilt.urdf)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${COMMON_MODELS}/300mm/${model}")
  file(READ "${COMMON_MODELS}/300mm/${model}" ROBOT_URDF)
  string(REPLACE "filename=\"meshes/" "filename=\"package://topoarm_description/models/300mm/meshes/" ROBOT_URDF "${ROBOT_URDF}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${model}" "${ROBOT_URDF}")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${model}" DESTINATION share/${PROJECT_NAME}/models/300mm)
endforeach()
install(DIRECTORY "${COMMON_MODELS}/300mm/meshes" DESTINATION share/${PROJECT_NAME}/models/300mm
  FILES_MATCHING PATTERN "*.stl")

ament_package()
