cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include directory for header files (expects headers in include/vff_avoidance/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# dependenciess
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Create the executable by including both main and node implementation files
add_executable(avoidance_node 
  src/avoidance_vff_main.cpp 
  src/vff_avoidance/AvoidanceNode.cpp
)

ament_target_dependencies(avoidance_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
)


install(TARGETS avoidance_node DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
