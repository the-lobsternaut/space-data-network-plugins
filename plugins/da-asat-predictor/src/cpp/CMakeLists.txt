cmake_minimum_required(VERSION 3.14)
project(da_asat_predictor_plugin CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Plugin library
add_library(da_asat STATIC
    src/propagator.cpp
    src/interceptor.cpp
    src/engagement.cpp
)
target_include_directories(da_asat PUBLIC include)

# Tests
add_executable(test_da_asat tests/test_da_asat.cpp)
target_link_libraries(test_da_asat da_asat)
target_include_directories(test_da_asat PRIVATE include)
