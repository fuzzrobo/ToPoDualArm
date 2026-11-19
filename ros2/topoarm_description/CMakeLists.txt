cmake_minimum_required(VERSION 3.5)
project(topoarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ../../config/ DESTINATION share/${PROJECT_NAME}/config)


# 正本はルートmodels/。ROS用URIへの変換結果はビルドディレクトリだけに置く。
set(COMMON_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/../../models")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${COMMON_MODELS}/300mm/robot.urdf")
file(READ "${COMMON_MODELS}/300mm/robot.urdf" ROBOT_URDF)
string(REPLACE "filename=\"meshes/" "filename=\"package://topoarm_description/models/300mm/meshes/" ROBOT_URDF "${ROBOT_URDF}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/robot.urdf" "${ROBOT_URDF}")
install(DIRECTORY "${COMMON_MODELS}/" DESTINATION share/${PROJECT_NAME}/models
  PATTERN "robot.urdf" EXCLUDE)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/robot.urdf" DESTINATION share/${PROJECT_NAME}/models/300mm)

ament_package()
