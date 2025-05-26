cmake_minimum_required(VERSION 3.8)
project(sami_ttt)

find_package(ament_cmake REQUIRED)

install(DIRECTORY behaviorbank/
  DESTINATION share/${PROJECT_NAME}/behaviorbank)
  
ament_package()
