cmake_minimum_required(VERSION 2.8.3)
project(innopolis_vtol_dynamics)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    mav_msgs
    sensor_msgs
    geometry_msgs
    uavcan_msgs
    uavcan_communicator
    tf2
    tf2_ros
    geographiclib_conversions
    message_generation
)

find_package(mavlink)

catkin_package(
    LIBRARIES innopolis_vtol_dynamics
    CATKIN_DEPENDS roscpp std_msgs mav_msgs sensor_msgs geometry_msgs tf2 tf2_ros roslib message_runtime
)


###########
## Build ##
###########
include_directories(
    include
    include/dynamics
    ${catkin_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dynamics/vtolDynamicsSim.cpp
                            src/dynamics/flightgogglesDynamicsSim.cpp
                            src/dynamics/uavDynamicsSimBase.cpp
                            libs/multicopterDynamicsSim/inertialMeasurementSim.cpp
                            libs/multicopterDynamicsSim/multicopterDynamicsSim.cpp
                            src/sensors.cpp
)

## 1. Declare a C++ innopolis_vtol_dynamics_node executable
add_executable(${PROJECT_NAME}_node src/innopolis_vtol_dynamics_node.cpp)
target_include_directories(${PROJECT_NAME}_node
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## 2. Declare a C++ mavlink_communicator executable
add_executable(${PROJECT_NAME}_mavlink_communicator src/mavlink_communicator.cpp)
target_include_directories(${PROJECT_NAME}_mavlink_communicator
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_mavlink_communicator PROPERTIES OUTPUT_NAME mavlink_communicator PREFIX "")
add_dependencies(${PROJECT_NAME}_mavlink_communicator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mavlink_communicator
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## 3. Declare a C++ inno_vtol_reverse_mixer_node executable
add_executable(${PROJECT_NAME}_inno_vtol_reverse_mixer_node src/inno_vtol_reverse_mixer_node.cpp)
target_include_directories(${PROJECT_NAME}_inno_vtol_reverse_mixer_node
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_inno_vtol_reverse_mixer_node PROPERTIES OUTPUT_NAME inno_vtol_reverse_mixer_node PREFIX "")
add_dependencies(${PROJECT_NAME}_inno_vtol_reverse_mixer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_inno_vtol_reverse_mixer_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test src/tests/test_vtol_dynamics.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}-test
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
endif()
