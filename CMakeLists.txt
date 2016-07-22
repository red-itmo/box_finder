cmake_minimum_required(VERSION 2.8.3)
project(box_finder)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        roscpp
        nav_msgs
        tf
        actionlib
        move_base_msgs
        message_generation
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_service_files(
    FILES
    GetGoal.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(
    move_to_table
    src/move_to_table.cpp
    src/MoveToTableNode.cpp
    src/RobotClass.cpp
    src/TrueAngle.cpp
)
target_link_libraries(
    move_to_table
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


add_executable(
    automapping
    src/automapping.cpp
    src/AutomappingNode.cpp
    src/RobotClass.cpp
    src/TrueAngle.cpp
)
target_link_libraries(
    automapping
    ${catkin_LIBRARIES}
)


add_executable(
    exploration_server
    src/exploration.cpp
    src/Explorer.cpp
)
target_link_libraries(
    exploration_server
    ${catkin_LIBRARIES}
)
