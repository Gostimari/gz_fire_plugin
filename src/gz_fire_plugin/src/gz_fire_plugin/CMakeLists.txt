cmake_minimum_required(VERSION 3.22)
project(gz_fire_plugin VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic packages (Ubuntu 24.04 pe ye names hain)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(FirePlugin SHARED
  src/FirePlugin.cc
)

target_include_directories(FirePlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(FirePlugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  sdformat14::sdformat14
)

set_target_properties(FirePlugin PROPERTIES
  OUTPUT_NAME FirePlugin
  PREFIX      "lib"
)

install(TARGETS FirePlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
