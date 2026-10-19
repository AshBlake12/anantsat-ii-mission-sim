cmake_minimum_required(VERSION 3.16)
project(cubesat_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()

add_executable(cubesat_sim src/sim.cpp)
target_include_directories(cubesat_sim PRIVATE src)

add_executable(cubesat_tests tests/tests.cpp)
target_include_directories(cubesat_tests PRIVATE src)

enable_testing()
add_test(NAME physics COMMAND cubesat_tests)
