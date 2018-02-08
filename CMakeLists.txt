cmake_minimum_required(VERSION 3.0)
project(i2c)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(i2c main.cc)
add_executable(hello test.cpp)

target_link_libraries(i2c wiringPi pthread crypt rt)
target_link_libraries(hello pthread)