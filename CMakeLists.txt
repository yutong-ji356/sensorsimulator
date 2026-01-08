#CMakeLists.txt

#this file is a boilerplate for how files combine, how they compile and what they depend on
cmake_minimum_required(VERSION 3.14)

project(SensorSim VERSION 0.1.0 LANGUAGES CXX) #set version #, declares as c++ language
set(CMAKE_CXX_STANDARD 18)
set(CMAKE_CXX_STANDARD_REQUIRED ON) #requires specific support c++17
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")#type of compiler
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native) #different warnings
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")#type of compiler
    add_compile_options(/W4 /O2) #different warnings
endif()

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) #type is more optimized, debugging is harder than debug option. checks so no accidental unoptimized binary
endif()

#for configuration, text for choices made
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# Options/switches for different tasks
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(USE_OPENMP "Enable OpenMP parallelization" OFF)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE) #fails if not found
message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files/ setting structure
set(CORE_SOURCES
    src/core/trajectory.cpp
)

set(UTILS_SOURCES
    src/utils/quaternion.cpp
)

set(SENSOR_SOURCES
    src/sensors/imu.cpp
    src/sensors/gps.cpp
)

set(FUSION_SOURCES
    src/fusion/complementary.cpp
    src/fusion/ekf.cpp
)

# Create library
add_library(sensorsim STATIC
    ${CORE_SOURCES}
    ${UTILS_SOURCES}
    ${SENSOR_SOURCES}
    ${FUSION_SOURCES}
)

target_link_libraries(sensorsim PUBLIC Eigen3::Eigen) #link eigen to library just created

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Installation
install(TARGETS sensorsim
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)