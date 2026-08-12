cmake_minimum_required(VERSION 3.16)
project(automotive_ecu_simulator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Main executable
# ---------------------------------------------------------------------------
add_executable(ecu_simulator src/main.cpp)
target_include_directories(ecu_simulator PRIVATE include)
target_link_libraries(ecu_simulator PRIVATE Threads::Threads)

if(MSVC)
  target_compile_options(ecu_simulator PRIVATE /W4)
else()
  target_compile_options(ecu_simulator PRIVATE -Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# Tests (Google Test, fetched automatically)
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  add_executable(ecu_tests tests/test_can_bus.cpp)
  target_include_directories(ecu_tests PRIVATE include)
  target_link_libraries(ecu_tests PRIVATE gtest_main Threads::Threads)

  include(GoogleTest)
  gtest_discover_tests(ecu_tests)
endif()
