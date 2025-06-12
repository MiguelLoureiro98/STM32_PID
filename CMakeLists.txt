cmake_minimum_required(VERSION 3.28.3)

project(libPID VERSION 0.1.0 LANGUAGES C)

find_library(MATH m)

if(MATH)

    add_library(PID STATIC src/PID.c)
    target_include_directories(PID PUBLIC inc/)
    target_link_libraries(PID PRIVATE ${MATH})
    message(STATUS "Math library found and linked sucessfully.")

else()

    message(FATAL_ERROR "Math library not found!")

endif()

add_subdirectory(tests)