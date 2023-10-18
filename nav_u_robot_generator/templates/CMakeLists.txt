cmake_minimum_required(VERSION 3.5)
project([PACKAGE_NAME])
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf/
        DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()
