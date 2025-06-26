cmake_minimum_required(VERSION 3.8)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (แนะนำสำหรับ ROS2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

# Create executable
add_executable(my_bot src/my_bot.cpp)

# Target dependencies (วิธีที่ถูกต้องใน ROS2)
ament_target_dependencies(my_bot
  rclcpp
  hardware_interface
  ros2_control
  ros2_controllers
  controller_manager
)

# Include directories (เฉพาะเมื่อมีโฟลเดอร์ include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_include_directories(my_bot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

# Install executable
install(TARGETS my_bot
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install header files (เฉพาะเมื่อมีโฟลเดอร์ include)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()