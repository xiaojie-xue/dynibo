cmake_minimum_required(VERSION 3.16)
project(dyno VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(DYNO_CARGO_PROFILE "release" CACHE STRING "Cargo profile used for the native library")
option(DYNO_BUILD_EXAMPLES "Build C and C++ usage examples" OFF)

if(DYNO_CARGO_PROFILE STREQUAL "release")
    set(DYNO_CARGO_PROFILE_FLAG --release)
    set(DYNO_CARGO_TARGET_DIR "release")
else()
    set(DYNO_CARGO_PROFILE_FLAG --profile ${DYNO_CARGO_PROFILE})
    set(DYNO_CARGO_TARGET_DIR "${DYNO_CARGO_PROFILE}")
endif()

if(WIN32)
    set(DYNO_LIBRARY_FILE "dyno_c.dll")
    set(DYNO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
    if(MSVC)
        set(DYNO_IMPLIB_FILE "dyno_c.dll.lib")
    else()
        set(DYNO_IMPLIB_FILE "libdyno_c.dll.a")
    endif()
elseif(APPLE)
    set(DYNO_LIBRARY_FILE "libdyno_c.dylib")
    set(DYNO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
else()
    set(DYNO_LIBRARY_FILE "libdyno_c.so")
    set(DYNO_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
endif()

set(DYNO_CARGO_LIBRARY
    "${CMAKE_CURRENT_SOURCE_DIR}/target/${DYNO_CARGO_TARGET_DIR}/${DYNO_LIBRARY_FILE}")
set(DYNO_BUILT_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/${DYNO_LIBRARY_FILE}")

add_custom_command(
    OUTPUT "${DYNO_BUILT_LIBRARY}"
    COMMAND "${CARGO_EXECUTABLE}" build -p dyno-c ${DYNO_CARGO_PROFILE_FLAG}
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${DYNO_CARGO_LIBRARY}" "${DYNO_BUILT_LIBRARY}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS
        Cargo.toml Cargo.lock
        bindings/c/Cargo.toml bindings/c/src/lib.rs
        bindings/c/include/dyno/dyno.h bindings/c/include/dyno/dyno.hpp
        src/lib.rs src/error.rs src/model.rs src/robot.rs
        src/robot/workspace.rs src/spatial.rs src/urdf.rs
    COMMENT "Building dyno C ABI with Cargo"
    VERBATIM)

add_custom_target(dyno_c_build ALL DEPENDS "${DYNO_BUILT_LIBRARY}")
add_library(dyno_c SHARED IMPORTED GLOBAL)
set_target_properties(dyno_c PROPERTIES
    IMPORTED_LOCATION "${DYNO_BUILT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/bindings/c/include")
if(WIN32)
    set(DYNO_CARGO_IMPLIB
        "${CMAKE_CURRENT_SOURCE_DIR}/target/${DYNO_CARGO_TARGET_DIR}/${DYNO_IMPLIB_FILE}")
    set_target_properties(dyno_c PROPERTIES IMPORTED_IMPLIB "${DYNO_CARGO_IMPLIB}")
endif()
add_dependencies(dyno_c dyno_c_build)
add_library(dyno::dyno ALIAS dyno_c)

if(DYNO_BUILD_EXAMPLES)
    add_executable(dyno_c_example bindings/c/examples/basic.c)
    target_link_libraries(dyno_c_example PRIVATE dyno::dyno)
    add_executable(dyno_cpp_example bindings/c/examples/basic.cpp)
    target_compile_features(dyno_cpp_example PRIVATE cxx_std_17)
    target_link_libraries(dyno_cpp_example PRIVATE dyno::dyno)
endif()

configure_package_config_file(
    bindings/c/cmake/dyno-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/dyno-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dyno")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/dyno-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
configure_file(bindings/c/cmake/dyno.pc.in
               "${CMAKE_CURRENT_BINARY_DIR}/dyno.pc" @ONLY)

install(FILES "${DYNO_BUILT_LIBRARY}"
        DESTINATION "${DYNO_LIBRARY_INSTALL_DIR}")
if(WIN32)
    install(FILES "${DYNO_CARGO_IMPLIB}"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif()
install(DIRECTORY bindings/c/include/dyno
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/dyno-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/dyno-config-version.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dyno")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/dyno.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

set(CPACK_PACKAGE_NAME "dyno")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "dyno contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Tree-structured robot kinematics and dynamics C/C++ package")
set(CPACK_GENERATOR "TGZ")
include(CPack)
