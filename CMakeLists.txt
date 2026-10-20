cmake_minimum_required(VERSION 3.16)

project(ulixarm_api LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

add_executable(ulixarm_api
    src/main.cpp
    src/ulixarm.cpp
    src/protocol.cpp
)

target_include_directories(ulixarm_api 
PRIVATE 
${LIBSERIALPORT_INCLUDE_DIRS}
)

target_include_directories(ulixarm_api
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include/ulixarm_api
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${LIBSERIALPORT_LIBRARIES}
    Eigen3::Eigen
    pinocchio::pinocchio
)