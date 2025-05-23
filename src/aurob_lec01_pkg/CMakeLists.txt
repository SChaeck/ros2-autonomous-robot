cmake_minimum_required(VERSION 3.5)
project(aurob_lec01_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#build
## Build options for publisher node
## Configure exe file, target link library, and additional dependency
add_executable(lec01_publisher src/lec01_publisher.cpp)
ament_target_dependencies(lec01_publisher rclcpp std_msgs)

## Build options for subscriber node
add_executable(lec01_subscriber src/lec01_subscriber.cpp)
ament_target_dependencies(lec01_subscriber rclcpp std_msgs)


#install
install(TARGETS
  lec01_publisher
  lec01_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
