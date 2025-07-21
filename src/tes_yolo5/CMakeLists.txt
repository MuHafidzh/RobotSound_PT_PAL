cmake_minimum_required(VERSION 3.8)
project(yolov5_detection)

set(CMAKE_CXX_STANDARD 17)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Find packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCNN REQUIRED ncnn)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${NCNN_INCLUDE_DIRS})

# Add executable
add_executable(yolov5_detection yolov5.cpp)

# Link libraries (tambah OpenMP)
target_link_libraries(yolov5_detection 
    ${OpenCV_LIBS} 
    ${NCNN_LIBRARIES}
    ncnn
    OpenMP::OpenMP_CXX
    gomp
)

# Compiler flags
target_compile_options(yolov5_detection PRIVATE ${NCNN_CFLAGS_OTHER})