cmake_minimum_required(VERSION 3.8)
project(auv_max_teleoperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/auv_teleop_keyboard.cpp"
)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

set(EXEC_NAME "auv_teleop_keyboard")

add_executable(${EXEC_NAME} src/auv_teleop_keyboard.cpp)
target_link_libraries(${EXEC_NAME} ${PROJECT_NAME}_lib ncurses)
ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})


install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/gui_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_package()
