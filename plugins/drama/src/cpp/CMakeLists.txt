cmake_minimum_required(VERSION 3.16)
project(drama VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Source files ─────────────────────────────────────────────────────────────
set(DRAMA_SOURCES
    src/types.cpp
    src/oscar.cpp
    src/ares.cpp
    src/croc.cpp
    src/sara.cpp
    src/master.cpp
    src/compliance.cpp
)

set(DRAMA_HEADERS
    include/types.h
    include/constants.h
)

# ── Library ──────────────────────────────────────────────────────────────────
add_library(drama_lib STATIC ${DRAMA_SOURCES} ${DRAMA_HEADERS})
target_include_directories(drama_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Native test executable ───────────────────────────────────────────────────
add_executable(test_drama tests/test_drama.cpp)
target_link_libraries(test_drama PRIVATE drama_lib)

# ── WASM build (Emscripten) ──────────────────────────────────────────────────
if(EMSCRIPTEN)
    add_executable(drama_wasm
        ${DRAMA_SOURCES}
        ../wasm_api.cpp
    )
    target_include_directories(drama_wasm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    set_target_properties(drama_wasm PROPERTIES
        OUTPUT_NAME "drama"
        SUFFIX ".js"
    )

    target_link_options(drama_wasm PRIVATE
        "SHELL:-s EXPORTED_FUNCTIONS=['_compute_lifetime_json','_compute_flux_json','_compute_collision_risk_json','_compute_cross_section_json','_compute_average_cross_section_json','_analyze_reentry_json','_get_environment_json','_check_compliance_json','_malloc','_free']"
        "SHELL:-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','UTF8ToString','stringToUTF8','lengthBytesUTF8']"
        "SHELL:-s MODULARIZE=1"
        "SHELL:-s EXPORT_NAME='DRAMAModule'"
        "SHELL:-s ALLOW_MEMORY_GROWTH=1"
        "SHELL:-s INITIAL_MEMORY=16777216"
        "SHELL:-s ENVIRONMENT='web,worker'"
        "SHELL:-s NO_EXIT_RUNTIME=1"
        "SHELL:-s ASSERTIONS=0"
        "SHELL:-s SINGLE_FILE=0"
    )
endif()

# ── CTest ────────────────────────────────────────────────────────────────────
enable_testing()
add_test(NAME drama_tests COMMAND test_drama)
