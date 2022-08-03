cmake_minimum_required(VERSION 3.5)
project(flex_nav_turtlebot3_demo_bringup)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
###########
## Build ##
###########


#############
## Install ##
#############


# Mark anything (useful) else for installation
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY paths
        DESTINATION share/${PROJECT_NAME}
)


ament_package()
