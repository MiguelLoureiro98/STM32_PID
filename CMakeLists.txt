cmake_minimum_required(VERSION 3.28.3)

project(STMPID_lib VERSION 0.1.0 LANGUAGES C)

add_library(PID STATIC src/PID.c)
target_include_directories(PID PUBLIC inc/)
target_link_libraries(PID PUBLIC m)

include(CTest)

add_executable(test_lib tests/test_getters_setters.c)
target_link_libraries(test_lib PUBLIC PID)
target_link_libraries(test_lib PUBLIC criterion)

add_test(NAME test_lib COMMAND test_lib)