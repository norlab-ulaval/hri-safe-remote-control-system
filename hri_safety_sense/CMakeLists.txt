cmake_minimum_required(VERSION 2.8.3)
project(hri_safety_sense)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        sensor_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/JoystickHandler.cpp
        src/SerialInterface.c
        src/VehicleInterface.c
        src/VscProcess.cpp
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hri_safety_sense
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
)

add_executable(safe_remote_control src/main.cpp)
target_link_libraries(safe_remote_control
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

 
