cmake_minimum_required(VERSION 3.5)
project(gamepad_teleoperation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
