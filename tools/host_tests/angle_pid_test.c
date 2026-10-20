/*
 * Host-side unit test for Algorithm/Src/angle_pid.c.
 *
 * Build:
 *   cc -std=c11 -I tools/host_tests/shim -I stm32_ws/Algorithm/Src \
 *      tools/host_tests/angle_pid_test.c \
 *      stm32_ws/Algorithm/Src/angle_pid.c \
 *      stm32_ws/Algorithm/Src/pid_core.c \
 *      stm32_ws/Algorithm/Src/pid_angle.c \
 *      stm32_ws/Algorithm/Src/pid.c -lm -o /tmp/angle_pid_test
 */

#include "angle_pid.h"

#include <math.h>
#include <stdio.h>

static int failures;

static void check(int cond, const char *name)
{
    if (cond) {
        printf("PASS %s\n", name);
    } else {
        printf("FAIL %s\n", name);
        failures++;
    }
}

static int near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

static angle_pid_cfg_t base_cfg(void)
{
    angle_pid_cfg_t cfg = {
        .kp = 1.0F,
        .ki = 0.0F,
        .kd = 0.0F,
        .max_out = 10.0F,
        .max_iout = 10.0F,
        .deadband = 0.0F,
        .freeze_err = 0.0F,
        .freeze_d = 0.0F,
        .active_err = 3.1415926F,
        .leak = 1.0F,
    };
    return cfg;
}

int main(void)
{
    angle_pid_t pid;
    angle_pid_cfg_t cfg = base_cfg();
    float out;
    int i;

    check(near(angle_pid_wrap(0.0F), 0.0F, 1e-6F), "wrap(0)");
    check(near(angle_pid_wrap(3.14159274F), -3.14159274F, 1e-5F), "wrap(pi)");
    check(near(angle_pid_wrap(3.0F * 3.1415926F), -3.1415926F, 1e-5F), "wrap(3pi)");
    check(near(angle_pid_wrap(10.0F * 3.1415926F), 0.0F, 1e-4F), "wrap(10pi)");

    check(angle_pid_init(&pid, &cfg) == PID_CORE_OK, "init");
    angle_pid_set_target(&pid, 3.0F);
    out = angle_pid_calc(&pid, -3.0F);
    check(near(out, -0.2831853F, 1e-4F), "short path 3 -> -3");
    check(near(angle_pid_get_target(&pid), 3.0F, 1e-6F), "target unchanged after short path");

    angle_pid_set_target(&pid, -3.0F);
    out = angle_pid_calc(&pid, 3.0F);
    check(near(out, 0.2831853F, 1e-4F), "short path -3 -> 3");

    angle_pid_set_target(&pid, 3.0F);
    for (i = 0; i < 1000; i++) {
        (void)angle_pid_calc(&pid, -3.0F);
    }
    check(near(angle_pid_get_target(&pid), 3.0F, 1e-6F), "target unchanged after 1000 steps");

    cfg = base_cfg();
    cfg.deadband = 0.05F;
    cfg.ki = 1.0F;
    check(angle_pid_init(&pid, &cfg) == PID_CORE_OK, "init deadband");
    angle_pid_set_target(&pid, 0.0F);
    out = angle_pid_calc(&pid, 0.01F);
    check(near(out, 0.0F, 1e-6F), "deadband output 0");
    check(pid.in_deadband == 1U, "deadband flag set");
    out = angle_pid_calc(&pid, -0.2F);
    check(near(out, 0.4F, 1e-4F), "integral frozen in deadband");
    check(near(angle_pid_get_target(&pid), 0.0F, 1e-6F), "target unchanged in deadband");

    cfg = base_cfg();
    cfg.kp = 100.0F;
    cfg.max_out = 0.5F;
    check(angle_pid_init(&pid, &cfg) == PID_CORE_OK, "init limit");
    angle_pid_set_target(&pid, 1.0F);
    out = angle_pid_calc(&pid, 0.0F);
    check(near(out, 0.5F, 1e-6F), "output limit");

    cfg = base_cfg();
    cfg.kd = 1.0F;
    check(angle_pid_init(&pid, &cfg) == PID_CORE_OK, "init d_in");
    angle_pid_set_target(&pid, 0.1F);
    out = angle_pid_calc_d(&pid, 0.0F, 2.0F);
    check(near(out, 2.1F, 1e-4F), "external derivative");

    out = angle_pid_calc(&pid, NAN);
    check(near(out, 0.0F, 1e-6F), "NaN input returns 0");

    printf("failures=%d\n", failures);
    return (failures == 0) ? 0 : 1;
}
