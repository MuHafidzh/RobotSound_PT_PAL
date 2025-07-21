cmake_minimum_required(VERSION 3.8)
project(device)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

find_library(GPIOD_LIBRARY gpiodcxx)
if(NOT GPIOD_LIBRARY)
    message(FATAL_ERROR "libgpiod not found")
endif()

find_library(MODBUS_LIBRARY modbus)
if(NOT MODBUS_LIBRARY)
    message(FATAL_ERROR "libmodbus not found")
endif()

# GPIO Node
add_executable(gpio_node src/gpio.cpp)
ament_target_dependencies(gpio_node rclcpp std_msgs)
target_link_libraries(gpio_node ${GPIOD_LIBRARY})

# Motor Node (PERBAIKAN: nama executable harus sama dengan launch file)
add_executable(zlac8015_node src/zlac8015d.cpp)
ament_target_dependencies(zlac8015_node rclcpp std_msgs geometry_msgs)

# Laser Node
add_executable(laser_node src/laser.cpp)
ament_target_dependencies(laser_node rclcpp std_msgs)
target_link_libraries(laser_node ${MODBUS_LIBRARY})

# Install executables
install(TARGETS
  gpio_node
  zlac8015_node  # Pastikan nama ini sama dengan yang di launch file
  laser_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()