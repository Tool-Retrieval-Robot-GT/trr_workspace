cmake_minimum_required(VERSION 3.5)
project(trr_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY description launch rviz worlds
    DESTINATION share/${PROJECT_NAME}/
)

install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)

install(PROGRAMS scripts/remapper.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()