cmake_minimum_required(VERSION 3.0)
project(i2c)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

set(default_build_type "Release")

add_executable(i2c main.cc I2CDevices/GyroI2C.cpp I2CDevices/GyroI2C.h I2CDevices/AccelerometerI2C.cpp I2CDevices/AccelerometerI2C.h)
add_executable(hello test.cpp)

target_link_libraries(i2c wiringPi pthread crypt rt)
target_link_libraries(hello pthread)