cmake_minimum_required(VERSION 3.5)
project(hri_safety_sense)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
        include
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/EmergencyStop.srv"
    "srv/KeyString.srv"
    "srv/KeyValue.srv"
    DEPENDENCIES std_msgs sensor_msgs
)

#add_library(${PROJECT_NAME}
#        src/JoystickHandler.cpp
#        src/SerialInterface.c
#        src/VehicleInterface.c
#        src/VscProcess.cpp
#)

#target_link_libraries(${PROJECT_NAME}
#        rclcpp
#)

#add_executable(safe_remote_control src/main.cpp)
#target_link_libraries(safe_remote_control
#        ${PROJECT_NAME}
#        rclcpp
#)

# Install targets
#install(TARGETS ${PROJECT_NAME} safe_remote_control
#    DESTINATION lib/${PROJECT_NAME}
#)

# Package
ament_package()


 
