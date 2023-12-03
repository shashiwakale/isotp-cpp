cmake_minimum_required(VERSION  3.5)

#Set C++ std to 14
set (CMAKE_CXX_STANDARD 14)

#Project name
project(isotp15765)

#Get src files
FILE(GLOB SRC ./*.cpp)

#Add include directories
include_directories(../include)

#Create static lib
add_library(${PROJECT_NAME} STATIC ${SRC})

#Link libraries
target_link_libraries(${PROJECT_NAME} pthread)

#Enable compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)