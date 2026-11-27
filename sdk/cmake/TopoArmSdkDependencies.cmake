# Ubuntu 22.04のyaml-cpp 0.7は名前空間付きターゲットを公開しない。
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# Ubuntu 22.04のTinyXML2開発パッケージはCMake設定を同梱しない。
if(NOT TARGET tinyxml2::tinyxml2)
  find_package(tinyxml2 CONFIG QUIET)
endif()
if(NOT TARGET tinyxml2::tinyxml2)
  find_path(TOPOARM_TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)
  find_library(TOPOARM_TINYXML2_LIBRARY tinyxml2 REQUIRED)
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    IMPORTED_LOCATION "${TOPOARM_TINYXML2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${TOPOARM_TINYXML2_INCLUDE_DIR}")
endif()
