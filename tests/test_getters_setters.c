#include "PID.h"
#include <criterion/criterion.h>

TestSuite(getset_tests);

Test(getset_tests, init){

    PID controller;

    PID_init(&controller, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0);

    cr_assert(get_Kp(&controller) == 0.0, "Failed to initialise Kp.");

}