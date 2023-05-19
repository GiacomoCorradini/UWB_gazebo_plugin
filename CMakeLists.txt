cmake_minimum_required(VERSION 3.8)
project(uwb_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
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
find_package(rosmsgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${ament_cmake_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gtec_uwb_plugin SHARED src/UwbPlugin.cpp)
target_include_directories(gtec_uwb_plugin PUBLIC include)
ament_target_dependencies(gtec_uwb_plugin
  "gazebo_ros_pkgs"
  "std_msgs"
  "visualization_msgs"
  "rosmsgs"
  "tf2"
  "gazebo_ros"
)
ament_export_libraries(gtec_uwb_plugin)

ament_export_include_directories(include)

install(
  TARGETS gtec_uwb_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
