cmake_minimum_required(VERSION 3.10)
project(Pi_Arm_Code)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") # For C++

# Include directories
include_directories(include)

# Find Eigen
find_package(Eigen3 REQUIRED)
find_library(PIGPIO_LIBRARY pigpio REQUIRED)

# Find source files
file(GLOB SOURCES "src/*.cpp")

# Create executable
add_executable(pi_arm ${SOURCES})

# Include Eigen headers
target_include_directories(pi_arm PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(pi_arm ${PIGPIO_LIBRARY})

# Installation rules
install(TARGETS pi_arm DESTINATION "executables")