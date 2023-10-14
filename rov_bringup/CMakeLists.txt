cmake_minimum_required(VERSION 3.8)
project(rov_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS
  #scripts/dump_rosout.py
  #scripts/mission_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_auto_package(INSTALL_TO_SHARE behavior_trees config launch params)
ament_auto_package(INSTALL_TO_SHARE config launch param)
