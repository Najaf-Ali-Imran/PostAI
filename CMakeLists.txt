cmake_minimum_required(VERSION 3.15)

# Set vcpkg toolchain file (for local Windows development)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

project(PostAI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define Windows version (Windows 10) - only on Windows
if(WIN32)
    add_compile_definitions(_WIN32_WINNT=0x0A00)
endif()

# Check if using custom Crow location (Docker/Linux build)
if(DEFINED CROW_INCLUDE_DIR)
    message(STATUS "Using Crow from: ${CROW_INCLUDE_DIR}")
    include_directories(${CROW_INCLUDE_DIR})
    
    add_executable(PostAI main.cpp)
    
    find_package(Threads REQUIRED)
    target_link_libraries(PostAI PRIVATE Threads::Threads)
else()
    # Use vcpkg Crow (Windows local development)
    find_package(Crow CONFIG REQUIRED)
    
    add_executable(PostAI main.cpp)
    target_link_libraries(PostAI PRIVATE Crow::Crow)
endif()