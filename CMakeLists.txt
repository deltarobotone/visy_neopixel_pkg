cmake_minimum_required(VERSION 2.8.3)
project(visy_neopixel_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rospy)

catkin_python_setup()

add_message_files(FILES Neopixel.msg Neopixels.msg)
add_service_files(DIRECTORY srv FILES LightRing.srv StatusBar.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs rospy)

catkin_install_python(PROGRAMS
  scripts/status_bar_node.py
  scripts/light_ring_node.py
  scripts/neo_pixel_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
