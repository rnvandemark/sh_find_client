cmake_minimum_required(VERSION 3.5)
project(sh_find_client)

find_package(rclcpp REQUIRED)
find_package(sh_find_common REQUIRED)

include_directories(include)

add_executable(find_client src/find_client.cpp)
ament_target_dependencies(find_client
    rclcpp
    sh_find_common
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS find_client
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
