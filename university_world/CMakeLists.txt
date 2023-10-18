cmake_minimum_required(VERSION 3.5)
project(university_world)
find_package(ament_cmake REQUIRED)
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY maps/
        DESTINATION share/${PROJECT_NAME}/maps
)
install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
