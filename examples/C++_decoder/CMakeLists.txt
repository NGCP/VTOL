cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR D:\\OpenCV\\opencv-4.1.2\\build\\install)
set(OPENCV_ENABLE_NONFREE)

project (C++_decoder)
find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable (runtime runtime.cpp)
target_link_libraries(runtime ${OpenCV_LIBS})
