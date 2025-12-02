cmake_minimum_required(VERSION 3.8)
project(vinspect_rviz_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(ROS2_DEPS
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
  std_msgs
  std_srvs
  vinspect_msgs
)

foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

set(CMAKE_AUTOMOC ON)

set(SRC_FILES
  src/settings_panel.cpp
  src/area_panel.cpp
  src/status_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${ROS2_DEPS})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(${ROS2_DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()