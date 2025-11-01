#include <stdio.h>
#include "PI_controller.h"
#include "motor_model.h"

int main()
{
    // Simulation step time
    const float Ts = 0.0001f;

    // Motor and controller instances
    PMSM_Model motor;
    PMSM_Init(&motor);

    PI_Controller pi_d, pi_q;
    PI_Init(&pi_d, 2.0f, 400.0f, 0.05f, Ts, -200.0f, 200.0f);
    PI_Init(&pi_q, 2.0f, 400.0f, 0.05f, Ts, -200.0f, 200.0f);

    // Simulation setup
    float id_ref = 0.0f;
    float iq_ref = 2.0f;   // torque-producing current
    float Tl = 0.0f;       // load torque

    printf("Step\tid\t iq\t vd\t vq\t omega(rad/s)\n");

    for (int step = 0; step < 5000; step++)
    {
        // Feedforward decoupling terms
        float vff_d = motor.omega_m * motor.Lq * motor.iq;                      // +ωLq*iq
        float vff_q = -motor.omega_m * (motor.Ld * motor.id + motor.psi_f);     // -ω(Ld*id + ψf)

        // PI control for d and q
        float vd = PI_Update(&pi_d, id_ref, motor.id, vff_d);
        float vq = PI_Update(&pi_q, iq_ref, motor.iq, vff_q);

        // Update PMSM model
        PMSM_Update(&motor, vd, vq, Tl, Ts);

        // Print values occasionally
        if (step % 500 == 0)
            printf("%4d\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                   step, motor.id, motor.iq, vd, vq, motor.omega_m);

        // Change iq reference after 2500 steps
        if (step == 2500)
            iq_ref = 4.0f;
    }

    return 0;
}
