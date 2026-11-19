# 通信なしの床・首の補正。実機の目標量子化は上位セッションから注入する。
add_library(topoarm_safety OBJECT floor_guard.cpp target_limiter.cpp)
target_include_directories(topoarm_safety PRIVATE ../include)
target_link_libraries(topoarm_safety PRIVATE tinyxml2::tinyxml2 Eigen3::Eigen)
target_compile_features(topoarm_safety PRIVATE cxx_std_17)
set_target_properties(topoarm_safety PROPERTIES POSITION_INDEPENDENT_CODE ON)
