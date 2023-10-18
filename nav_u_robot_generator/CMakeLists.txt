cmake_minimum_required(VERSION 3.8)
project(nav_u_robot_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/nav_u_robot_generator/robot_gen_widget.hpp
  include/nav_u_robot_generator/app_widget.hpp
)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/app_widget.cpp
    src/robot_gen_widget.cpp
    src/templates.cpp
    ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  Qt5Core
  Qt5Widgets
  rclcpp
  rcl_interfaces
)
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY templates/
        DESTINATION share/${PROJECT_NAME}/templates
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)

ament_package()
