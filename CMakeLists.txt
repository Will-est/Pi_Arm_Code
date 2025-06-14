cmake_minimum_required(VERSION 3.10)
project(Pi_Arm_Code)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Find source files
file(GLOB SOURCES "src/*.cpp")

# Create executable
add_executable(pi_arm ${SOURCES})

# Include Eigen headers
target_include_directories(pi_arm PRIVATE ${EIGEN3_INCLUDE_DIR})

# Installation rules
install(TARGETS pi_arm DESTINATION bin)