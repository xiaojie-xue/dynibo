cmake_minimum_required(VERSION 3.16)
project(dynibo VERSION 0.3.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(DYNIBO_CARGO_PROFILE "release" CACHE STRING "Cargo profile used for the native library")
option(DYNIBO_BUILD_EXAMPLES "Build C and C++ usage examples" OFF)

if(DYNIBO_CARGO_PROFILE STREQUAL "release")
    set(DYNIBO_CARGO_PROFILE_FLAG --release)
    set(DYNIBO_CARGO_TARGET_DIR "release")
else()
    set(DYNIBO_CARGO_PROFILE_FLAG --profile ${DYNIBO_CARGO_PROFILE})
    set(DYNIBO_CARGO_TARGET_DIR "${DYNIBO_CARGO_PROFILE}")
endif()

if(WIN32)
    set(DYNIBO_LIBRARY_FILE "dynibo_c.dll")
    set(DYNIBO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
    if(MSVC)
        set(DYNIBO_IMPLIB_FILE "dynibo_c.dll.lib")
    else()
        set(DYNIBO_IMPLIB_FILE "libdynibo_c.dll.a")
    endif()
elseif(APPLE)
    set(DYNIBO_LIBRARY_FILE "libdynibo_c.dylib")
    set(DYNIBO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
else()
    set(DYNIBO_LIBRARY_FILE "libdynibo_c.so")
    set(DYNIBO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
endif()

set(DYNIBO_CARGO_LIBRARY
    "${CMAKE_CURRENT_SOURCE_DIR}/target/${DYNIBO_CARGO_TARGET_DIR}/${DYNIBO_LIBRARY_FILE}")
set(DYNIBO_BUILT_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/${DYNIBO_LIBRARY_FILE}")

add_custom_command(
    OUTPUT "${DYNIBO_BUILT_LIBRARY}"
    COMMAND "${CARGO_EXECUTABLE}" build -p dynibo-c ${DYNIBO_CARGO_PROFILE_FLAG}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${DYNIBO_CARGO_LIBRARY}" "${DYNIBO_BUILT_LIBRARY}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS
        Cargo.toml Cargo.lock
        bindings/c/Cargo.toml bindings/c/src/lib.rs
        bindings/c/include/dynibo/dynibo.h bindings/c/include/dynibo/dynibo.hpp
        src/lib.rs src/error.rs src/model.rs
        src/model/joint.rs src/model/link.rs src/model/tree.rs
        src/robot.rs src/robot/workspace.rs src/spatial.rs
    COMMENT "Building dynibo C ABI with Cargo"
    VERBATIM)

add_custom_target(dynibo_c_build ALL DEPENDS "${DYNIBO_BUILT_LIBRARY}")
add_library(dynibo_c SHARED IMPORTED GLOBAL)
set_target_properties(dynibo_c PROPERTIES
    IMPORTED_LOCATION "${DYNIBO_BUILT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/bindings/c/include")
if(WIN32)
    set(DYNIBO_CARGO_IMPLIB
        "${CMAKE_CURRENT_SOURCE_DIR}/target/${DYNIBO_CARGO_TARGET_DIR}/${DYNIBO_IMPLIB_FILE}")
    set_target_properties(dynibo_c PROPERTIES IMPORTED_IMPLIB "${DYNIBO_CARGO_IMPLIB}")
endif()
add_dependencies(dynibo_c dynibo_c_build)
add_library(dynibo::dynibo ALIAS dynibo_c)

if(DYNIBO_BUILD_EXAMPLES)
    add_executable(dynibo_c_example examples/c/franka.c)
    target_link_libraries(dynibo_c_example PRIVATE dynibo::dynibo)
    add_executable(dynibo_cpp_example bindings/c/examples/basic.cpp)
    target_compile_features(dynibo_cpp_example PRIVATE cxx_std_17)
    target_link_libraries(dynibo_cpp_example PRIVATE dynibo::dynibo)
endif()

configure_package_config_file(
    bindings/c/cmake/dynibo-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/dynibo-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dynibo")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/dynibo-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
configure_file(bindings/c/cmake/dynibo.pc.in
               "${CMAKE_CURRENT_BINARY_DIR}/dynibo.pc" @ONLY)

install(FILES "${DYNIBO_BUILT_LIBRARY}"
        DESTINATION "${DYNIBO_LIBRARY_INSTALL_DIR}")
if(WIN32)
    install(FILES "${DYNIBO_CARGO_IMPLIB}"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif()
install(DIRECTORY bindings/c/include/dynibo
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/dynibo-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/dynibo-config-version.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dynibo")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/dynibo.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

set(CPACK_PACKAGE_NAME "dynibo")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "dynibo contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Tree-structured robot kinematics and dynamics C/C++ package")
set(CPACK_GENERATOR "TGZ")
include(CPack)
