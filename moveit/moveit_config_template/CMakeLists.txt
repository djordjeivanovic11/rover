cmake_minimum_required(VERSION 3.8)
project(moveit_config_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
endif()

# Install srdf directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/srdf)
  install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME}/)
endif()

# Install rviz directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 