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
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_robot_model.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX     ROBOT_MODEL_NX
#define NZ     ROBOT_MODEL_NZ
#define NU     ROBOT_MODEL_NU
#define NP     ROBOT_MODEL_NP
#define NBX    ROBOT_MODEL_NBX
#define NBX0   ROBOT_MODEL_NBX0
#define NBU    ROBOT_MODEL_NBU
#define NSBX   ROBOT_MODEL_NSBX
#define NSBU   ROBOT_MODEL_NSBU
#define NSH    ROBOT_MODEL_NSH
#define NSG    ROBOT_MODEL_NSG
#define NSPHI  ROBOT_MODEL_NSPHI
#define NSHN   ROBOT_MODEL_NSHN
#define NSGN   ROBOT_MODEL_NSGN
#define NSPHIN ROBOT_MODEL_NSPHIN
#define NSBXN  ROBOT_MODEL_NSBXN
#define NS     ROBOT_MODEL_NS
#define NSN    ROBOT_MODEL_NSN
#define NG     ROBOT_MODEL_NG
#define NBXN   ROBOT_MODEL_NBXN
#define NGN    ROBOT_MODEL_NGN
#define NY0    ROBOT_MODEL_NY0
#define NY     ROBOT_MODEL_NY
#define NYN    ROBOT_MODEL_NYN
#define NH     ROBOT_MODEL_NH
#define NPHI   ROBOT_MODEL_NPHI
#define NHN    ROBOT_MODEL_NHN
#define NPHIN  ROBOT_MODEL_NPHIN
#define NR     ROBOT_MODEL_NR


int main()
{

    robot_model_solver_capsule *acados_ocp_capsule = robot_model_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = ROBOT_MODEL_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = robot_model_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("robot_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = robot_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = robot_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = robot_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = robot_model_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = robot_model_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = robot_model_acados_get_nlp_opts(acados_ocp_capsule);

    // initial condition
    int idxbx0[NBX0];
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;

    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = 0;
    ubx0[0] = 0;
    lbx0[1] = 0;
    ubx0[1] = 0;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;

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

    for (int ii = 0; ii <= N; ii++)
    {
        robot_model_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = robot_model_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("robot_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("robot_model_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    robot_model_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = robot_model_acados_free(acados_ocp_capsule);
    if (status) {
        printf("robot_model_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = robot_model_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("robot_model_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
