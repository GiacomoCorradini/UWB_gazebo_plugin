cmake_minimum_required(VERSION 3.8)
project(UWB_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gtec_rosmsgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${ament_cmake_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gtec_uwb_plugin SHARED src/UwbPlugin.cpp)
ament_target_dependencies(gtec_uwb_plugin
  gazebo_ros_pkgs
  std_msgs
  visualization_msgs
  gtec_rosmsgs
  tf2
  gazebo_ros
)

install(
  TARGETS gtec_uwb_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# add_library(gtec_uwb_plugin src/UwbPlugin.cpp)
# target_link_libraries(gtec_uwb_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(gtec_uwb_plugin ${catkin_EXPORTED_TARGETS})

# add_library(gtec_magnetic_distortion_plugin src/MagneticDistortionPlugin.cpp)
# target_link_libraries(gtec_magnetic_distortion_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(gtec_magnetic_distortion_plugin ${catkin_EXPORTED_TARGETS})

# add_library(custom_skid_steer_drive src/CustomSkidSteerDrive.cpp src/CustomSkidSteerDrive.h)
# target_link_libraries(custom_skid_steer_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(custom_skid_steer_drive ${catkin_EXPORTED_TARGETS})

# add_library(gtec_tag_pos_publisher_plugin src/TagPosPublisherPlugin.cpp)
# target_link_libraries(gtec_tag_pos_publisher_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(gtec_tag_pos_publisher_plugin ${catkin_EXPORTED_TARGETS})

# ament_package(
#   DEPENDS
#     rclcpp
#     gtec_rosmsgs
#     gazebo_ros
# )

ament_package()
