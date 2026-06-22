cmake_minimum_required(VERSION 3.8)
project(quac_inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics rclcpp geometry_msgs trajectory_msgs std_msgs)

install(TARGETS
  inverse_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
