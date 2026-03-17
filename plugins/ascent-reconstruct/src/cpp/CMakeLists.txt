cmake_minimum_required(VERSION 3.14)
project(ascent_reconstruct_plugin CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ascent_reconstruct STATIC
    src/trajectory.cpp
    src/estimator.cpp
)
target_include_directories(ascent_reconstruct PUBLIC include)

add_executable(test_ascent_reconstruct tests/test_ascent_reconstruct.cpp)
target_link_libraries(test_ascent_reconstruct ascent_reconstruct)
target_include_directories(test_ascent_reconstruct PRIVATE include)
