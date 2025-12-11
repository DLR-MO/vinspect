cmake_minimum_required(VERSION 3.8)
project(vinspect_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sparse.msg"
  "msg/Settings.msg"
  "msg/AreaData.msg"
  "msg/Status.msg"
  "srv/StartReconstruction.srv"
  "srv/SaveDICONDE.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
