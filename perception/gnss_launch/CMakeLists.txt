cmake_minimum_required(VERSION 3.5)
project(gnss_launch)

find_package(ament_cmake REQUIRED)

# Install launch and config files
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  launch/gnss_health_monitor.py
  launch/gnss_validator.py
  launch/gnss_setup.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
