cmake_minimum_required(VERSION 3.8)
project(mc_radar_sdk LANGUAGES CXX)

find_package(Threads REQUIRED)

add_library(mc_radar_sdk INTERFACE)
add_library(mc_radar_sdk::mc_radar_sdk ALIAS mc_radar_sdk)
target_include_directories(mc_radar_sdk INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(mc_radar_sdk INTERFACE cxx_std_11)
target_link_libraries(mc_radar_sdk INTERFACE Threads::Threads)

add_executable(radar_test_main tests/test_main.cpp)
target_link_libraries(radar_test_main PRIVATE mc_radar_sdk)
target_compile_options(radar_test_main PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_custom_target(radar_viewer
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/view_points.py
          --receiver $<TARGET_FILE:radar_test_main>
  DEPENDS radar_test_main
  USES_TERMINAL
)

include(CTest)
if(BUILD_TESTING)
  add_executable(radar_sdk_test tests/radar_sdk_test.cpp)
  target_link_libraries(radar_sdk_test PRIVATE mc_radar_sdk)
  target_compile_options(radar_sdk_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME radar_sdk_test COMMAND radar_sdk_test)
  set_tests_properties(radar_sdk_test PROPERTIES TIMEOUT 30)
endif()
