cmake_minimum_required(VERSION 3.0.2)
project(ros_multi_tb3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

# 添加以下行來安裝 Python 腳本
catkin_install_python(PROGRAMS
  
  scripts/robot_position_filter.py
  
  scripts/get_robot_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 如果您有其他 Python 腳本，也可以在這裡添加它們
# 例如：
# scripts/other_script.py
