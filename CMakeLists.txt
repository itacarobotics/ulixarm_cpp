cmake_minimum_required(VERSION 3.16)

project(ulixarm_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

add_executable(ulixarm_cpp
src/main.cpp
src/ulixarm.cpp
src/protocol.cpp
)

target_include_directories(ulixarm_cpp 
PRIVATE 
${LIBSERIALPORT_INCLUDE_DIRS}
)

target_include_directories(ulixarm_cpp
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ulixarm_cpp PRIVATE ${LIBSERIALPORT_LIBRARIES})
target_link_libraries(ulixarm_cpp PRIVATE Eigen3::Eigen)