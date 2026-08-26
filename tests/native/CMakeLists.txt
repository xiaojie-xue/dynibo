cmake_minimum_required(VERSION 3.16)
project(dynibo_package_tests LANGUAGES C CXX)

if(NOT DEFINED DYNIBO_TEST_URDF OR NOT EXISTS "${DYNIBO_TEST_URDF}")
    message(FATAL_ERROR "DYNIBO_TEST_URDF must name an existing URDF")
endif()
if(NOT DEFINED DYNIBO_TEST_REFERENCE OR NOT EXISTS "${DYNIBO_TEST_REFERENCE}")
    message(FATAL_ERROR "DYNIBO_TEST_REFERENCE must name an existing reference corpus")
endif()

find_package(dynibo CONFIG REQUIRED)
enable_testing()

add_executable(dynibo_c_package_test test_c.c)
target_link_libraries(dynibo_c_package_test PRIVATE dynibo::dynibo)
add_test(NAME c_package COMMAND dynibo_c_package_test
         "${DYNIBO_TEST_URDF}" "${DYNIBO_TEST_REFERENCE}")

add_executable(dynibo_cpp_package_test test_cpp.cpp)
target_compile_features(dynibo_cpp_package_test PRIVATE cxx_std_17)
target_link_libraries(dynibo_cpp_package_test PRIVATE dynibo::dynibo)
add_test(NAME cpp_package COMMAND dynibo_cpp_package_test
         "${DYNIBO_TEST_URDF}" "${DYNIBO_TEST_REFERENCE}")
