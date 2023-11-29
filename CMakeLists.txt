# cmake_minimum_required(VERSION 3.5)
# project(my_bot)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(
#   DIRECTORY config description launch scripts worlds
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(my_bot)
# Default to C++17
set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# find_package(your_package_msgs REQUIRED)  # Assuming your service definition is named your_package_msgs

# Include directories
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS} 
)

# Add executable
# set(SRC_FILES
#     src/waypoints_server.cpp
# )

# Add the executable
add_executable(waypoints_server src/waypoints_server.cpp)
add_executable(waypoints_client src/waypoints_client.cpp)
add_executable(nav2_handler src/nav2_handler.cpp)

# Link libraries
ament_target_dependencies(waypoints_server
    rclcpp
    sensor_msgs
    tutorial_interfaces
    cv_bridge
    OpenCV
)

ament_target_dependencies(waypoints_client
    rclcpp
    nav_msgs
    tutorial_interfaces
)

ament_target_dependencies(nav2_handler
    rclcpp
    nav_msgs
)

target_link_libraries(waypoints_server
  ${OpenCV_LIBS}
)
# Install
install(TARGETS
    waypoints_server
    waypoints_client
    nav2_handler
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config description launch worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
