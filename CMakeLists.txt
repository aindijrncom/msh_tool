cmake_minimum_required(VERSION 3.16)
project(msh_tool VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Static linking for MinGW (self-contained .exe, no DLL dependency)
if(MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++ -static")
endif()

# nlohmann/json (header-only)
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE include)

# MSH Tool executable
add_executable(msh_tool
    src/main.cpp
    src/lexer.cpp
    src/parser.cpp
    src/validator.cpp
    src/query_engine.cpp
)

target_include_directories(msh_tool PRIVATE src include)
target_link_libraries(msh_tool PRIVATE nlohmann_json)

# Optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(msh_tool PRIVATE -O3 -march=native)
endif()

# Warnings
if(MSVC)
    target_compile_options(msh_tool PRIVATE /W4 /utf-8)
else()
    target_compile_options(msh_tool PRIVATE -Wall -Wextra -Wpedantic)
endif()
