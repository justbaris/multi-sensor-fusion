cmake_minimum_required(VERSION 3.16)
project(MultiSensorFusionEngine)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)
find_package(Eigen3 REQUIRED)

add_executable(fusion
    src/main.cpp
    src/sensors/imu.cpp
    src/sensors/gps.cpp
    src/filters/ekf.cpp
)

target_link_libraries(fusion Eigen3::Eigen)