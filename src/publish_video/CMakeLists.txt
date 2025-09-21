cmake_minimum_required(VERSION 3.8)
project(publish_video)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configurations for clangd
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # this line is required because ROS2 always defaults to c++17. But clangd wasn't defaulting to this c++17 and was looking into older libraries and not finding some functions when importing rclcpp. This now just makes sure that c++17 is used
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(publish_video_node src/publish_video.cpp)
ament_target_dependencies(publish_video_node rclcpp cv_bridge sensor_msgs)
target_link_libraries (publish_video_node ${OpenCV_LIBRARIES})

install (TARGETS
        publish_video_node
        DESTINATION "lib/${PROJECT_NAME}"
)

ament_package()
