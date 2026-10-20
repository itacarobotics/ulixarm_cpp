cmake_minimum_required(VERSION 3.16)

project(ulixarm_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

pkg_check_modules(LIBSERIALPORT REQUIRED IMPORTED_TARGET libserialport)

# Core library
add_library(ulixarm_core
    src/ulixarm.cpp
    src/protocol.cpp
)

set_target_properties(ulixarm_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(ulixarm_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ulixarm_cpp
)

target_link_libraries(ulixarm_core PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    PkgConfig::LIBSERIALPORT
)

# Example executable
add_executable(gravity
    examples/gravity.cpp
)

target_link_libraries(gravity PRIVATE ulixarm_core)