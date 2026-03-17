cmake_minimum_required(VERSION 3.14)
project(domain_awareness_pipeline CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(da_pipeline STATIC
    src/notam_parser.cpp
    src/ntm_parser.cpp
    src/exclusion_zone.cpp
    src/sgp4.cpp
    src/tle_correlator.cpp
    src/trajectory.cpp
    src/ascent_estimator.cpp
    src/engagement.cpp
    src/pipeline.cpp
)
target_include_directories(da_pipeline PUBLIC include)

add_executable(test_da_pipeline tests/test_da_pipeline.cpp)
target_link_libraries(test_da_pipeline da_pipeline)
target_include_directories(test_da_pipeline PRIVATE include)
