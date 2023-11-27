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
set(CMAKE_CXX_STANDARD 17)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(your_package_msgs REQUIRED)  # Assuming your service definition is named your_package_msgs

# Include directories
include_directories(include)

# Add executable
add_executable(image_processor
    scripts/image_processor.cpp
)

# Link libraries
ament_target_dependencies(image_processor
    rclcpp
    sensor_msgs
    # your_package_msgs
)

# Install
install(TARGETS
    image_processor
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
