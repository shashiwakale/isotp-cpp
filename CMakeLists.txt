cmake_minimum_required(VERSION  3.5)

#Set C++ std to 14
set (CMAKE_CXX_STANDARD 14)

#Project name
project(isotp_sample)

#Get src files
FILE(GLOB SRC ./src/*.cpp)

#Add include directories
include_directories(./include)

#Create executable
add_executable(${PROJECT_NAME} ${SRC})

#Link libraries
target_link_libraries(${PROJECT_NAME} pthread)

#Enable compiler flags
target_compile_options(isotp_sample PRIVATE -Wall -Wextra -Wpedantic)