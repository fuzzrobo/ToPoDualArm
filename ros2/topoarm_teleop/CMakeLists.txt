cmake_minimum_required(VERSION 3.5)
project(topoarm_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS外でも使用するPythonモジュールはリポジトリの共通実装をそのまま入れる。
ament_python_install_package(topoarm_leader_follower
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../python/teleop/topoarm_leader_follower")

install(PROGRAMS topoarm_teleop/leader_follower_node.py DESTINATION lib/${PROJECT_NAME} RENAME leader_follower_node)

ament_package()
