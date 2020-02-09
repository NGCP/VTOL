cmake_minimum_required(VERSION 3.1)
project (NordenBombsight)

#enable c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "D:\\Intel RealSense SDK 2.0\\lib\\x64")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "D:\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "D:\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "D:\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$$\(Configuration\)")  # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "D:\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
endif()

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
# getting-started


SET(OpenCV_DIR D:\\OpenCV\\opencv-4.1.2\\build\\install)
set(OPENCV_ENABLE_NONFREE)
find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}, ${ADDITIONAL_INCLUDE_DIRS})

add_executable (NordenBombsight NordenBombsight.cpp)


#this needs to have only OpenCV_LIBS in it
target_link_libraries(NordenBombsight PRIVATE ${OpenCV_LIBS} ${REALSENSE2_FOUND})
set_target_properties (NordenBombsight PROPERTIES FOLDER Examples)


#try manually adding realsense2.dll to build folder (next to Nordenbombsight.exe)

if(WIN32)
    message(STATUS "Adding Post build script to realsense.dll to project's binary folder")
    message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
    add_custom_command(TARGET NordenBombsight POST_BUILD #adds post-build event
        COMMAND ${CMAKE_COMMAND} -E copy_if_different    #which executes "cmake -E copy_if_different..."
        "${DLL_DIR}/realsense2.dll"                      #<- in-file 
        ${PROJECT_BINARY_OUTPUT_PATH})                   #<- out-file path
endif()

install(
    TARGETS

    NordenBombsight

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

add_executable (runtime runtime.cpp)
target_link_libraries(runtime ${OpenCV_LIBS})

add_executable (calibrate calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})