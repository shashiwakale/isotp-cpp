cmake_minimum_required(VERSION  3.5)

#Set C++ std to 14
set (CMAKE_CXX_STANDARD 14)

#Project name
project(isotp_sample)

#Add include directories
include_directories(./include)

#add sub directory
add_subdirectory(src)

#Create executable
add_executable(${PROJECT_NAME} main.cpp)

#Link libraries
target_link_libraries(${PROJECT_NAME} pthread isotp15765)

#Enable compiler flags
target_compile_options(isotp_sample PRIVATE -Wall -Wextra -Wpedantic)