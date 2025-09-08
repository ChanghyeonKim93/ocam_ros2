message("   -- [DIRECTORY]: " core)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("/usr/include/opencv4/")

set(SRC ${SRC} withrobot_camera.cpp withrobot_utility.cpp)

add_executable(ocam_ros2_node ocam_ros2_node.cc ${SRC})
target_link_libraries(
  ocam_ros2_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  -lv4l2
  -ludev)
ament_target_dependencies(ocam_ros2_node rclcpp sensor_msgs geometry_msgs
                          message_filters)
install(TARGETS ocam_ros2_node DESTINATION lib/${PROJECT_NAME})
