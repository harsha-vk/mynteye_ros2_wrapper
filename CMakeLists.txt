cmake_minimum_required(VERSION 3.5)
project(mynteye_ros2_wrapper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mynteye REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${SDK_DIR}/src)

# Create Cpp executable
add_executable(mynteye_raw_data src/mynteye_raw_data.cpp)
target_link_libraries(mynteye_raw_data ${OpenCV_LIBS} mynteye)
ament_target_dependencies(mynteye_raw_data rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge)

# Install Cpp executables
install(TARGETS
  mynteye_raw_data
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/display_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Config Directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
