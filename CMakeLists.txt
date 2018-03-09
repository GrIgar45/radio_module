cmake_minimum_required(VERSION 3.0)
project(i2c)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE "Release")
message("Build type is " ${CMAKE_BUILD_TYPE})

find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.cc I2CDevices/GyroI2C.cpp I2CDevices/GyroI2C.h I2CDevices/AccelerometerI2C.cpp I2CDevices/AccelerometerI2C.h)
#add_executable(hello test.cpp)

target_link_libraries(${PROJECT_NAME} wiringPi ${CMAKE_THREAD_LIBS_INIT} crypt rt)
#target_link_libraries(hello pthread)