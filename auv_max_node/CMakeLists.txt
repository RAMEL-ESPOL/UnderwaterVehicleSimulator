cmake_minimum_required(VERSION 3.8)
project(auv_max_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/cmd_vel_thrust_converter.cpp"
  "src/gazebo_rviz_synchronizer.cpp" 
  "src/sonar_topic_remapper.cpp"
  "src/node_main.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXEC_NAME "auv_main_node_main")

add_executable(${EXEC_NAME} src/node_main.cpp)
target_link_libraries(${EXEC_NAME} ${PROJECT_NAME}_lib ncurses)
ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})


install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)
ament_package()
