cmake_minimum_required(VERSION 3.8)
project(rov_teleoperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/control_profundidad.cpp"
  "src/rov_teleoperacion.cpp" 
  "src/control_posxy.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXEC_NAME "rov_teleoperacion")

add_executable(${EXEC_NAME} src/rov_teleoperacion.cpp)
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
