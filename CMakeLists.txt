cmake_minimum_required(VERSION 3.0.2)
project(uav_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  gazebo_msgs
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  Parcel.msg
  ParcelArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs tf
                 gazebo_msgs visualization_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/dispatcher
  scripts/courier
  scripts/pose_delivery_monitor
  scripts/make_city
  scripts/clearance_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds urdf rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
