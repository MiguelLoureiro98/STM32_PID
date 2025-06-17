#include "PID.h"
#include <criterion/criterion.h>

TestSuite(getset_tests);

Test(getset_tests, init){

    PID controller;

    PID_init(&controller, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0);

    cr_assert(get_Kp(&controller) == 0.0, "Failed to initialise Kp.\n");
    cr_assert(get_Ki(&controller) == 0.0, "Failed to initialise Ki.\n");
    cr_assert(get_Kd(&controller) == 0.0, "Failed to initialise Kd.\n");
    cr_assert(controller.tau == 0.0, "Failed to initialise tau.\n");
    cr_assert(controller.Ts == 1.0, "Failed to initialise Ts.\n");
    cr_assert(controller.umin == -1.0, "Failed to initialise umin.\n");
    cr_assert(controller.umax == 1.0, "Failed to initialise umax.");

}