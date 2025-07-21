cmake_minimum_required(VERSION 3.8)
project(camera)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Find NCNN (fix this part)
find_library(NCNN_LIBRARY ncnn REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# YOLO Node
add_executable(yolo_node src/yolov5.cpp)
ament_target_dependencies(yolo_node 
    rclcpp 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    image_transport
)
target_link_libraries(yolo_node 
    ${OpenCV_LIBS} 
    ${NCNN_LIBRARY}
    ${YAMLCPP_LIBRARIES}
    OpenMP::OpenMP_CXX
)

target_include_directories(yolo_node PRIVATE
  ${YAMLCPP_INCLUDE_DIRS}
)

install(TARGETS yolo_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install models folder (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
    install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()