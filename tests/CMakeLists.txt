include(CTest)

find_library(CRITERION criterion)

if(CRITERION)

    add_executable(test_lib test_getters_setters.c)
    target_link_libraries(test_lib PRIVATE PID)
    target_link_libraries(test_lib PRIVATE ${CRITERION})
    message(STATUS "Criterion found and linked successfully.")

    add_test(NAME test_lib COMMAND test_lib)

else()

    message(FATAL_ERROR "Criterion not found.")

endif()