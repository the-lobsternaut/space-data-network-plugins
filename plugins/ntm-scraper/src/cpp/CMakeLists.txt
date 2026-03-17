cmake_minimum_required(VERSION 3.14)
project(ntm_scraper_plugin CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ntm_scraper STATIC
    src/parser.cpp
    src/archive.cpp
)
target_include_directories(ntm_scraper PUBLIC include)

add_executable(test_ntm_scraper tests/test_ntm_scraper.cpp)
target_link_libraries(test_ntm_scraper ntm_scraper)
target_include_directories(test_ntm_scraper PRIVATE include)
