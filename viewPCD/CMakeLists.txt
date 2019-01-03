cmake_minimum_required(VERSION 2.8.3)
project(view)
find_package(PCL 1.2 REQUIRED) #配置PCL库
include_directories(${PCL_INCLUDE_DIRS}) #配置头文件路径..例#include
add_executable(test view.cpp)
target_link_libraries(test ${PCL_LIBRARIES}) #链接PCL库