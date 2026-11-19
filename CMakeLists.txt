cmake_minimum_required(VERSION 3.16)

project(ulixarm_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

pkg_check_modules(LIBSERIALPORT REQUIRED IMPORTED_TARGET libserialport)

set(ULIXARM_DESCRIPTION_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ulixarm_description/src/ulixarm_description")
set(ULIXARM_DESCRIPTION_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/ulixarm_cpp/ulixarm_description")

# Core library
add_library(ulixarm_core
    src/ulixarm.cpp
    src/protocol.cpp
)
add_library(ulixarm_cpp::ulixarm_core ALIAS ulixarm_core)

# URDF lookup fallbacks, see UlixArm::UlixArm()
target_compile_definitions(ulixarm_core PRIVATE
    ULIXARM_INSTALL_DESCRIPTION_PATH="${CMAKE_INSTALL_PREFIX}/${ULIXARM_DESCRIPTION_INSTALL_DIR}"
    ULIXARM_SOURCE_DESCRIPTION_PATH="${ULIXARM_DESCRIPTION_SOURCE_DIR}"
)

target_compile_options(ulixarm_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

set_target_properties(ulixarm_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(ulixarm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(ulixarm_core PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    PkgConfig::LIBSERIALPORT
)

# Install
install(TARGETS ulixarm_core
    EXPORT ulixarm_cppTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# only the URDFs are needed by the library (meshes and xacro sources are not loaded)
install(FILES
    ${ULIXARM_DESCRIPTION_SOURCE_DIR}/urdf/arm.urdf
    ${ULIXARM_DESCRIPTION_SOURCE_DIR}/urdf/arm_gripper.urdf
    ${ULIXARM_DESCRIPTION_SOURCE_DIR}/urdf/arm_gripper_camera.urdf
    DESTINATION ${ULIXARM_DESCRIPTION_INSTALL_DIR}/urdf
)

install(EXPORT ulixarm_cppTargets
    NAMESPACE ulixarm_cpp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ulixarm_cpp
)

configure_package_config_file(cmake/ulixarm_cppConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ulixarm_cppConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ulixarm_cpp
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ulixarm_cppConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ulixarm_cppConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ulixarm_cppConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ulixarm_cpp
)

# Examples
add_executable(examples_gravity examples/gravity.cpp)
target_link_libraries(examples_gravity PRIVATE ulixarm_core)
target_compile_options(examples_gravity PRIVATE -Wall -Wextra -Wpedantic -Werror)
