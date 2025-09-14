
cmake_minimum_required(VERSION 3.16)
project(slam_gui)

# C++17标准，与系统其他组件保持一致
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 编译选项
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(interface REQUIRED)

# Qt5依赖
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# PCL依赖
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

# VTK依赖（用于3D可视化）
find_package(VTK REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  CommonMath
  RenderingCore
  RenderingOpenGL2
  InteractionStyle
  RenderingAnnotation
  FiltersCore
  FiltersGeometry
  IOXML
  IOImage
  QUIET
)

# Eigen3依赖
find_package(Eigen3 REQUIRED)

# yaml-cpp依赖
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

# 头文件
set(HEADER_FILES
  include/slam_gui/main_window.h
  include/slam_gui/ros2_interface.h
  include/slam_gui/system_monitor.h
  include/slam_gui/config_manager.h
  include/slam_gui/visualization_widget.h
  include/slam_gui/control_panel.h
  include/slam_gui/log_viewer.h
)

# 源文件
set(SOURCE_FILES
  src/main.cpp
  src/main_window.cpp
  src/ros2_interface.cpp
  src/system_monitor.cpp
  src/config_manager.cpp
  src/visualization_widget.cpp
  src/control_panel.cpp
  src/log_viewer.cpp
)

# UI文件
set(UI_FILES
  ui/main_window.ui
)

# 资源文件
set(RESOURCE_FILES
  resources/resources.qrc
)

# 创建可执行文件
add_executable(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
  ${UI_FILES}
  ${RESOURCE_FILES}
)

# 依赖声明
set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  tf2_ros
  pcl_conversions
  interface
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::OpenGL
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  yaml-cpp
)

# ROS2依赖
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}/
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()