cmake_minimum_required(VERSION 3.8)
project(point_cloud_filter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(localizer REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Since we need to use localizer's DynamicObjectFilter, we need to include its source
# This is a workaround since localizer doesn't export a library
set(LOCALIZER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../localizer/src/localizers)
set(LOCALIZER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../localizer/src)

include_directories(${LOCALIZER_INCLUDE_DIR})

set(SRC_LIST 
    src/point_cloud_filter_bridge.cpp
    ${LOCALIZER_SRC_DIR}/commons.cpp
    ${LOCALIZER_SRC_DIR}/dynamic_object_filter.cpp)

add_executable(point_cloud_filter_bridge_node 
    src/point_cloud_filter_bridge_node.cpp 
    ${SRC_LIST})

ament_target_dependencies(point_cloud_filter_bridge_node
    rclcpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    message_filters
    tf2_ros
    tf2_geometry_msgs
    livox_ros_driver2
    interface
    pcl_conversions
    pcl_ros)

target_link_libraries(point_cloud_filter_bridge_node 
    ${PCL_LIBRARIES} 
    yaml-cpp)

install(TARGETS point_cloud_filter_bridge_node 
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config 
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()