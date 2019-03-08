cmake_minimum_required(VERSION 2.8.3)
project(wm_admitance)
SET(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    urdf
    sensor_msgs
    roscpp
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)
 find_package(cmake_modules REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp sensor_msgs urdf
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 # ${Eigen_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME}
        src/WMGravityModule.cpp
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Install
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES wm_admitance_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
