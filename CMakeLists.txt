cmake_minimum_required(VERSION 4.2)
project(estimation_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(estimation_lab src/main.cpp
        src/KalmanFilter.cpp
        include/Simulator.h
        include/MonteCarlo.h
        src/Simulator.cpp
        src/MonteCarlo.cpp
        include/Metrics.h
        src/Metrics.cpp
        src/EstimatorRunner.cpp
        include/EstimatorRunner.h
        include/ModelConfig.h
        include/ICConfig.h
        include/MonteCarloResult.h
        include/SimConfig.h
)

target_include_directories(estimation_lab
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(estimation_lab
        PRIVATE
        Eigen3::Eigen)

