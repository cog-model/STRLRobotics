/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_robot_model.h"

#define NX     ROBOT_MODEL_NX
#define NZ     ROBOT_MODEL_NZ
#define NU     ROBOT_MODEL_NU
#define NP     ROBOT_MODEL_NP


int main()
{
    int status = 0;
    robot_model_sim_solver_capsule *capsule = robot_model_acados_sim_solver_create_capsule();
    status = robot_model_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = robot_model_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = robot_model_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = robot_model_acados_get_sim_out(capsule);
    void *acados_sim_dims = robot_model_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0;
    x_current[3] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 100;
    p[1] = 100;
    p[2] = 100;
    p[3] = 100;
    p[4] = 100;
    p[5] = 100;
    p[6] = 100;
    p[7] = 100;
    p[8] = 100;
    p[9] = 100;
    p[10] = 100;
    p[11] = 100;
    p[12] = 100;
    p[13] = 100;
    p[14] = 100;
    p[15] = 100;
    p[16] = 100;
    p[17] = 100;
    p[18] = 100;
    p[19] = 100;
    p[20] = 100;
    p[21] = 100;
    p[22] = 100;
    p[23] = 100;
    p[24] = 100;
    p[25] = 100;
    p[26] = 100;
    p[27] = 100;
    p[28] = 100;
    p[29] = 100;
    p[30] = 100;
    p[31] = 100;
    p[32] = 100;
    p[33] = 100;
    p[34] = 100;
    p[35] = 100;
    p[36] = 100;
    p[37] = 100;
    p[38] = 100;
    p[39] = 100;
    p[40] = 100;
    p[41] = 100;
    p[42] = 100;
    p[43] = 100;
    p[44] = 100;
    p[45] = 100;
    p[46] = 100;
    p[47] = 100;
    p[48] = 100;
    p[49] = 100;
    p[50] = 100;
    p[51] = 100;
    p[52] = 100;
    p[53] = 100;
    p[54] = 100;
    p[55] = 100;
    p[56] = 100;
    p[57] = 100;
    p[58] = 100;
    p[59] = 100;
    p[60] = 100;
    p[61] = 100;
    p[62] = 100;
    p[63] = 100;
    p[64] = 100;
    p[65] = 100;
    p[66] = 100;
    p[67] = 100;
    p[68] = 100;
    p[69] = 100;
    p[70] = 100;
    p[71] = 100;
    p[72] = 100;
    p[73] = 100;
    p[74] = 100;
    p[75] = 100;
    p[76] = 100;
    p[77] = 100;
    p[78] = 100;
    p[79] = 100;

    robot_model_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = robot_model_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = robot_model_acados_sim_free(capsule);
    if (status) {
        printf("robot_model_acados_sim_free() returned status %d. \n", status);
    }

    robot_model_acados_sim_solver_free_capsule(capsule);

    return status;
}
