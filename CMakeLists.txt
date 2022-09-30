cmake_minimum_required(VERSION 3.0.2)
project(upower_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
)

catkin_install_python(PROGRAMS scripts/battery_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)

  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test()
endif()
