cmake_minimum_required(VERSION 3.8)
project(hri_safety_sense)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hri_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
        src/JoystickHandler.cpp
        src/SerialInterface.c
        src/VehicleInterface.c
        src/VscProcess.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    hri_interfaces
)

add_executable(safe_remote_control src/main.cpp)
target_link_libraries(safe_remote_control
        ${PROJECT_NAME}
)

ament_target_dependencies(safe_remote_control
    rclcpp
    std_msgs
    sensor_msgs
    hri_interfaces
)

# Install targets
install(TARGETS ${PROJECT_NAME} safe_remote_control
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    rosidl_default_runtime
    rclcpp
    std_msgs
    sensor_msgs
    hri_interfaces
    rosidl_default_runtime
)
# Package
ament_package()


 
