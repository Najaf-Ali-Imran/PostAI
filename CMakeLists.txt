cmake_minimum_required(VERSION 3.15)

# Set vcpkg toolchain file
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

project(PostAI)

set(CMAKE_CXX_STANDARD 17)

# Define Windows version (Windows 10)
add_compile_definitions(_WIN32_WINNT=0x0A00)

# Find Crow web framework
find_package(Crow CONFIG REQUIRED)

# Single main.cpp with modular design - 7 distinct modules
add_executable(PostAI main.cpp)

# Link Crow
target_link_libraries(PostAI PRIVATE Crow::Crow)