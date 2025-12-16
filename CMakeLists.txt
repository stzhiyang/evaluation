cmake_minimum_required(VERSION 3.0.2)
project(evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  gazebo_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/detection_adapter.py
  scripts/data_association.py
  scripts/plot_generator.py
  scripts/multi_algo_evaluator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
