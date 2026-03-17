cmake_minimum_required(VERSION 3.14)
project(tle_correlator_plugin CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(tle_correlator STATIC
    src/sgp4.cpp
    src/geometry.cpp
    src/correlator.cpp
)
target_include_directories(tle_correlator PUBLIC include)

add_executable(test_tle_correlator tests/test_tle_correlator.cpp)
target_link_libraries(test_tle_correlator tle_correlator)
target_include_directories(test_tle_correlator PRIVATE include)
