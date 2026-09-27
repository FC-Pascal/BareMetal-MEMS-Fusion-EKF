#include <math.h>
#include "mems_sensor_data.h"

#define SCB_CPACR  (*(volatile unsigned int *)0xE000ED88u)

static void Default_Handler(void) {
    while (1) { }
}

void Reset_Handler(void) {
    // 1. Enable FPU (CP10 and CP11 full access)
    SCB_CPACR |= (0xFu << 20);
    __asm volatile ("dsb sy\n\tisb sy" ::: "memory");

    // 2. Jump straight to main, bypassing the broken C-runtime library
    extern int main(void);
    main();
    while (1) { }
}

/* Vector table mapped cleanly into Flash at 0x08000000 */
const void *const __Vectors[16] __attribute__((used, section("RESET"))) = {
    (void *)0x20010000,         /* Initial Stack Pointer */
    (void *)Reset_Handler,      /* Reset Handler */
    (void *)Default_Handler,    /* NMI */
    (void *)Default_Handler,    /* HardFault */
    (void *)Default_Handler,    /* MemManage */
    (void *)Default_Handler,    /* BusFault */
    (void *)Default_Handler,    /* UsageFault */
    0, 0, 0, 0,                 /* Reserved */
    (void *)Default_Handler,    /* SVCall */
    (void *)Default_Handler,    /* Debug Monitor */
    0,                          /* Reserved */
    (void *)Default_Handler,    /* PendSV */
    (void *)Default_Handler     /* SysTick */
};

int main(void) {
    // Explicitly initialize state variables here so we don't rely on __main
    float state_angle = 0.0f;
    float state_bias = 0.0f;

    float P[2][2] = {
        {1.0f, 0.0f},
        {0.0f, 1.0f}
    };

    #define Q_ANGLE 0.001f  
    #define Q_BIAS  0.003f  
    #define R_MEAS  3.0f    

    float filtered_angles[DATA_LENGTH];
    float rmse_error = 0.0f;

    for (int i = 0; i < DATA_LENGTH; i++) {
        // --- 1. PREDICT STEP ---
        float rate = gyro_meas[i] - state_bias;
        state_angle += rate * DT;

        P[0][0] += DT * (DT * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
        P[0][1] -= DT * P[1][1];
        P[1][0] -= DT * P[1][1];
        P[1][1] += Q_BIAS * DT;

        // --- 2. UPDATE STEP ---
        float y = accel_meas[i] - state_angle;
        float S = P[0][0] + R_MEAS;

        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        state_angle += K[0] * y;
        state_bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        filtered_angles[i] = state_angle;
        float error = true_angle[i] - state_angle;
        rmse_error += (error * error);
    }

    rmse_error = sqrtf(rmse_error / DATA_LENGTH);

    return 0;
}