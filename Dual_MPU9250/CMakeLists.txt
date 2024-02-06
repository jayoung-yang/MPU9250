cmake_minimum_required(VERSION 3.10)
project(mpu9250)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(mpu9250 
	Dual_MPU9250_dual_Ladybug.cpp
    MPU9250.cpp
)	

target_link_libraries(mpu9250 pigpio)