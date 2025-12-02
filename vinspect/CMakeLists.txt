cmake_minimum_required(VERSION 3.15)
project(vinspect)

# TODO This is just currently necessary to make Open3D work on Ubuntu 24.04. Remove in the future
set(CMAKE_C_COMPILER "gcc-11")
set(CMAKE_CXX_COMPILER "g++-11")

# Build with release optimizations and debug symbols by default
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebug)
endif()

set(CMAKE_CXX_STANDARD 20)
set(PYBIND11_FINDPYTHON ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(Open3D_ROOT /usr/)

# TODO this is not available without ROS...
find_package(ament_cmake REQUIRED) 
find_package(ament_cmake_python REQUIRED)
find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(Python3 REQUIRED)
find_package(pybind11 REQUIRED)
# fixes a problem in ubuntu24.04
set_target_properties(pybind11::pybind11_headers PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
)
find_package(backward_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(Git REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rocksdb REQUIRED)
find_package(TBB REQUIRED)

# Make the git commit hash available in the code
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(GIT_COMMIT_HASH)
  add_definitions(-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\")
else()
  message(WARNING "Could not determine Git commit hash.")
endif()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/dense.proto)

list(APPEND CMAKE_MODULE_PATH "/usr/include/tbb/")

find_package(OpenMP)

if(OPENMP_FOUND)
  if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
  endif()
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) #ToDo probably better to have all .proto file in one folder and inlcude that instead of ${CMAKE_CURRENT_BINARY_DIR}
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${fmt_INCLUDE_DIRS})
set(SOURCES src/inspection.cpp src/utils.cpp src/sparse_mesh.cpp ${PROTO_SRCS} ${PROTO_HDRS})

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} TBB::tbb OpenMP::OpenMP_CXX rocksdb fmt::fmt ${PROTOBUF_LIBRARY} ${OpenCV_LIBS} ${fmt_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  Open3D
  rocksdb
  nlohmann_json
  fmt
)

pybind11_add_module(vinspect_py src/python_bindings.cpp)
ament_target_dependencies(vinspect_py PUBLIC
  pybind11
  Python3
  Open3D
  rocksdb
  nlohmann_json
  fmt
)
target_link_libraries(vinspect_py PUBLIC vinspect rocksdb fmt::fmt ${PROTOBUF_LIBRARY} ${OpenCV_LIBS} ${fmt_LIBRARIES})
set_target_properties(vinspect_py PROPERTIES LINKER_LANGUAGE CXX)

if(BUILD_TESTING)
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    include/Octree/*
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add a hook for colcon to add the Python library directory to the PYTHONPATH environment variable
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/vinspect/hook/python_path.dsv
  "prepend-non-duplicate;PYTHONPATH;lib/python3/dist-packages")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/vinspect/hook/python_path.dsv
  DESTINATION share/vinspect/hook)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(
  TARGETS vinspect_py
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
)

ament_export_dependencies(fmt)
ament_export_dependencies(nlohmann_json)
ament_export_dependencies(Open3D)
ament_export_dependencies(pybind11)
ament_export_dependencies(Python3)
ament_export_dependencies(TBB)
ament_export_libraries(rocksdb)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_include_directories(${CMAKE_CURRENT_BINARY_DIR}) #ToDo See line: 34
ament_package()
