#add_subdirectory(Dual_MPU9250)

cmake_minimum_required(VERSION 3.10)
project(MPU9250BasicAHRS)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(MPU9250BasicAHRS 
    MPU9250BasicAHRS.cpp
)	

target_link_libraries(MPU9250BasicAHRS pigpio)