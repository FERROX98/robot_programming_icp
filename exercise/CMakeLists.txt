cmake_minimum_required(VERSION 3.16)

project(nicp_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs 
    sensor_msgs
    visualization_msgs
    tf2_ros)

catkin_package(
    INCLUDE_DIRS
    "include"
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2_ros
)

add_library(map SHARED src/map.cpp)
target_include_directories(map PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(map PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_library(icp SHARED src/icp/eigen_icp_2d.cpp)
target_include_directories(icp PUBLIC include/icp)
target_link_libraries(icp PUBLIC Eigen3::Eigen)

# Create rules for the nicp library
# source files are:
# - src/nicp/eigen_nicp_2d.cpp
# - src/nicp/normal_estimator.cpp
# It depends on Eigen and icp
add_library(nicp SHARED src/nicp/eigen_nicp_2d.cpp src/nicp/normal_estimator.cpp)
target_include_directories(nicp PUBLIC ${catkin_INCLUDE_DIRS} include/nicp)
target_link_libraries(nicp ${catkin_LIBRARIES} Eigen3::Eigen icp)


# Create rules for the normal_localizer2d library
# source files are:
# - src/normal_localizer2d.cpp
# - src/ros_bridge.cpp
# It depends on ROS packages (catkin), Eigen, nicp and map
add_library(normal_localizer2d SHARED src/normal_localizer2d.cpp src/ros_bridge.cpp)
target_include_directories(normal_localizer2d PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(normal_localizer2d ${catkin_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBRARIES} nicp map)

# Create rules for the localizer_node (located in bin/localizer_node.cpp)
# It depends on ROS packages (catkin), Eigen, map and normal_localizer2d
add_executable(localizer_node bin/localizer_node.cpp)
target_include_directories(localizer_node PUBLIC include include/nicp include/icp ${catkin_INCLUDE_DIRS})
target_link_libraries(localizer_node ${catkin_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBRARIES} map normal_localizer2d)

# Once nicp and normal_localizer2d libraries are built
# you can uncomment the compilation of normal_viewer node
add_executable(normal_viewer bin/normal_viewer.cpp)
target_include_directories(normal_viewer PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(normal_viewer PRIVATE ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nicp normal_localizer2d)
