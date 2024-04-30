cmake_minimum_required(VERSION 3.0.2)
project(stretch_task_manager)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY nodes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
