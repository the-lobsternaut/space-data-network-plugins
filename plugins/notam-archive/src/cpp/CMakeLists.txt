cmake_minimum_required(VERSION 3.14)
project(notam_archive_plugin CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(notam_archive STATIC
    src/archive.cpp
    src/analyzer.cpp
)
target_include_directories(notam_archive PUBLIC include)

add_executable(test_notam_archive tests/test_notam_archive.cpp)
target_link_libraries(test_notam_archive notam_archive)
target_include_directories(test_notam_archive PRIVATE include)
