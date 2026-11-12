cmake_minimum_required(VERSION 3.8)
project(kickboard_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds models config maps rviz scripts
        DESTINATION share/${PROJECT_NAME})
ament_package()
