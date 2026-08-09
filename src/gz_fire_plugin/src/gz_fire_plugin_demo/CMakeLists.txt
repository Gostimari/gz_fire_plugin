cmake_minimum_required(VERSION 3.22)
project(gz_fire_plugin_demo)

find_package(ament_cmake REQUIRED)

# Install worlds and URDFs so ros2 launch can find them
install(DIRECTORY worlds urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
