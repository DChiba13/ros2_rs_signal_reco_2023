cmake_minimum_required(VERSION 3.8)
project(ros2_rs_signal_reco_2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui videoio)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_rs_interfaces REQUIRED)

#set target
set(NODES rs_signal_reco_sub)
foreach(target IN LISTS NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp OpenCV cv_bridge sensor_msgs ros2_rs_interfaces)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()


#install launch directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

#install configuration directory
install(
  DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/
)

ament_package()
