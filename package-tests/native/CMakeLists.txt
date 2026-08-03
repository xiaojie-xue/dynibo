cmake_minimum_required(VERSION 3.16)
project(dyno_package_tests LANGUAGES C CXX)

if(NOT DEFINED DYNO_TEST_URDF OR NOT EXISTS "${DYNO_TEST_URDF}")
    message(FATAL_ERROR "DYNO_TEST_URDF must name an existing URDF")
endif()

find_package(dyno CONFIG REQUIRED)
enable_testing()

add_executable(dyno_c_package_test test_c.c)
target_link_libraries(dyno_c_package_test PRIVATE dyno::dyno)
add_test(NAME c_package COMMAND dyno_c_package_test "${DYNO_TEST_URDF}")

add_executable(dyno_cpp_package_test test_cpp.cpp)
target_compile_features(dyno_cpp_package_test PRIVATE cxx_std_17)
target_link_libraries(dyno_cpp_package_test PRIVATE dyno::dyno)
add_test(NAME cpp_package COMMAND dyno_cpp_package_test "${DYNO_TEST_URDF}")
