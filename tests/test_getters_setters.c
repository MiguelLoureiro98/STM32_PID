#include "PID.h"
#include <criterion/criterion.h>
#include <criterion/parameterized.h>

TestSuite(getset_tests);

/*struct init_test_params {

    double Kp;
    double Ki;
    double Kd;
    double tau;
    double umin;
    double umax;
    double Ts;

};

ParameterizedTestParameters(getset_tests, init){

    static struct init_test_params params[2];
    size_t nb_params = sizeof(params) / sizeof(struct init_test_params);

    return cr_make_param_array(struct init_test_params, params, nb_params);

}*/

Test(getset_tests, init){

    PID controller;

    PID_init(&controller, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0);

    cr_assert(get_Kp(&controller) == 0.0, "Failed to initialise Kp.\n");
    cr_assert(get_Ki(&controller) == 0.0, "Failed to initialise Ki.\n");
    cr_assert(get_Kd(&controller) == 0.0, "Failed to initialise Kd.\n");
    cr_assert(controller.tau == 0.0, "Failed to initialise tau.\n");
    cr_assert(controller.Ts == 1.0, "Failed to initialise Ts.\n");
    cr_assert(controller.umin == -1.0, "Failed to initialise umin.\n");
    cr_assert(controller.umax == 1.0, "Failed to initialise umax.\n");
    cr_assert(controller.Tt == 0.0, "Tt - wrong default value\n.");

    cr_assert(controller.u == 0.0, "u does not default to 0.\n");
    cr_assert(controller.I == 0.0, "I does not default to 0.\n");
    cr_assert(controller.D == 0.0, "D does not default to 0.\n");
    cr_assert(controller.past_y == 0.0, "past_y does not default to 0.\n");
    cr_assert(controller.past_e == 0.0, "past_e does not default to 0.\n");
    cr_assert(controller.uv == 0.0, "uv does not default to 0.\n");
    cr_assert(controller.past_uv == 0.0, "past_uv does not default to 0.\n");

}