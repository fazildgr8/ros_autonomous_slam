cmake_minimum_required(VERSION 3.5)
project(ros_autonomous_slam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif() 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ament_python_install_package(${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(global_rrt_detector 
"rclcpp" 
"std_msgs" 
"rmw_implementation" 
"tf2_ros" 
"nav_msgs" 
"visualization_msgs")

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(local_rrt_detector  
"rclcpp" 
"std_msgs" 
"rmw_implementation" 
"tf2_ros" 
"nav_msgs" 
"visualization_msgs")

include_directories(include 
${Boost_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRECTORIES})

# target_include_directories(global_rrt_detector PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_include_directories(local_rrt_detector PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)

# add_custom_target(depfiles DEPENDS include/functions.hpp include/mtrand.hpp)
# add_dependencies(global_rrt_detector depfiles)
# add_dependencies(local_rrt_detector depfiles)
# target_link_libraries(global_rrt_detector 
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_link_libraries(local_rrt_detector 
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_link_libraries(global_rrt_detector
# ${Boost_INCLUDE_DIRS} 
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_link_libraries(local_rrt_detector 
# ${Boost_INCLUDE_DIRS}
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)

install(TARGETS
  global_rrt_detector
  local_rrt_detector
  # DIRECTORY include launch maps media msg nodes rviz scripts src world
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(DIRECTORY 
launch 
maps
media
rviz
worlds
DESTINATION share/${PROJECT_NAME}/)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(global_rrt_detector "${cpp_typesupport_target}")
target_link_libraries(local_rrt_detector "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()