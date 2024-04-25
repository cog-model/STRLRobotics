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
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "robot_model_model/robot_model_model.h"
#include "acados_sim_solver_robot_model.h"


// ** solver data **

robot_model_sim_solver_capsule * robot_model_acados_sim_solver_create_capsule()
{
    void* capsule_mem = malloc(sizeof(robot_model_sim_solver_capsule));
    robot_model_sim_solver_capsule *capsule = (robot_model_sim_solver_capsule *) capsule_mem;

    return capsule;
}


int robot_model_acados_sim_solver_free_capsule(robot_model_sim_solver_capsule * capsule)
{
    free(capsule);
    return 0;
}


int robot_model_acados_sim_create(robot_model_sim_solver_capsule * capsule)
{
    // initialize
    const int nx = ROBOT_MODEL_NX;
    const int nu = ROBOT_MODEL_NU;
    const int nz = ROBOT_MODEL_NZ;
    const int np = ROBOT_MODEL_NP;
    bool tmp_bool;

    
    double Tsim = 0.15;

    
    // explicit ode
    capsule->sim_forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_vde_adj_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_expl_ode_fun_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));

    capsule->sim_forw_vde_casadi->casadi_fun = &robot_model_expl_vde_forw;
    capsule->sim_forw_vde_casadi->casadi_n_in = &robot_model_expl_vde_forw_n_in;
    capsule->sim_forw_vde_casadi->casadi_n_out = &robot_model_expl_vde_forw_n_out;
    capsule->sim_forw_vde_casadi->casadi_sparsity_in = &robot_model_expl_vde_forw_sparsity_in;
    capsule->sim_forw_vde_casadi->casadi_sparsity_out = &robot_model_expl_vde_forw_sparsity_out;
    capsule->sim_forw_vde_casadi->casadi_work = &robot_model_expl_vde_forw_work;
    external_function_param_casadi_create(capsule->sim_forw_vde_casadi, np);

    capsule->sim_vde_adj_casadi->casadi_fun = &robot_model_expl_vde_adj;
    capsule->sim_vde_adj_casadi->casadi_n_in = &robot_model_expl_vde_adj_n_in;
    capsule->sim_vde_adj_casadi->casadi_n_out = &robot_model_expl_vde_adj_n_out;
    capsule->sim_vde_adj_casadi->casadi_sparsity_in = &robot_model_expl_vde_adj_sparsity_in;
    capsule->sim_vde_adj_casadi->casadi_sparsity_out = &robot_model_expl_vde_adj_sparsity_out;
    capsule->sim_vde_adj_casadi->casadi_work = &robot_model_expl_vde_adj_work;
    external_function_param_casadi_create(capsule->sim_vde_adj_casadi, np);

    capsule->sim_expl_ode_fun_casadi->casadi_fun = &robot_model_expl_ode_fun;
    capsule->sim_expl_ode_fun_casadi->casadi_n_in = &robot_model_expl_ode_fun_n_in;
    capsule->sim_expl_ode_fun_casadi->casadi_n_out = &robot_model_expl_ode_fun_n_out;
    capsule->sim_expl_ode_fun_casadi->casadi_sparsity_in = &robot_model_expl_ode_fun_sparsity_in;
    capsule->sim_expl_ode_fun_casadi->casadi_sparsity_out = &robot_model_expl_ode_fun_sparsity_out;
    capsule->sim_expl_ode_fun_casadi->casadi_work = &robot_model_expl_ode_fun_work;
    external_function_param_casadi_create(capsule->sim_expl_ode_fun_casadi, np);

    

    // sim plan & config
    sim_solver_plan_t plan;
    plan.sim_solver = ERK;

    // create correct config based on plan
    sim_config * robot_model_sim_config = sim_config_create(plan);
    capsule->acados_sim_config = robot_model_sim_config;

    // sim dims
    void *robot_model_sim_dims = sim_dims_create(robot_model_sim_config);
    capsule->acados_sim_dims = robot_model_sim_dims;
    sim_dims_set(robot_model_sim_config, robot_model_sim_dims, "nx", &nx);
    sim_dims_set(robot_model_sim_config, robot_model_sim_dims, "nu", &nu);
    sim_dims_set(robot_model_sim_config, robot_model_sim_dims, "nz", &nz);


    // sim opts
    sim_opts *robot_model_sim_opts = sim_opts_create(robot_model_sim_config, robot_model_sim_dims);
    capsule->acados_sim_opts = robot_model_sim_opts;
    int tmp_int = 3;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "newton_iter", &tmp_int);
    double tmp_double = 0;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "newton_tol", &tmp_double);
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "collocation_type", &collocation_type);

 
    tmp_int = 4;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "num_stages", &tmp_int);
    tmp_int = 1;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "num_steps", &tmp_int);
    tmp_bool = 0;
    sim_opts_set(robot_model_sim_config, robot_model_sim_opts, "jac_reuse", &tmp_bool);


    // sim in / out
    sim_in *robot_model_sim_in = sim_in_create(robot_model_sim_config, robot_model_sim_dims);
    capsule->acados_sim_in = robot_model_sim_in;
    sim_out *robot_model_sim_out = sim_out_create(robot_model_sim_config, robot_model_sim_dims);
    capsule->acados_sim_out = robot_model_sim_out;

    sim_in_set(robot_model_sim_config, robot_model_sim_dims,
               robot_model_sim_in, "T", &Tsim);

    // model functions
    robot_model_sim_config->model_set(robot_model_sim_in->model,
                 "expl_vde_forw", capsule->sim_forw_vde_casadi);
    robot_model_sim_config->model_set(robot_model_sim_in->model,
                 "expl_vde_adj", capsule->sim_vde_adj_casadi);
    robot_model_sim_config->model_set(robot_model_sim_in->model,
                 "expl_ode_fun", capsule->sim_expl_ode_fun_casadi);

    // sim solver
    sim_solver *robot_model_sim_solver = sim_solver_create(robot_model_sim_config,
                                               robot_model_sim_dims, robot_model_sim_opts);
    capsule->acados_sim_solver = robot_model_sim_solver;


    /* initialize parameter values */
    double* p = calloc(np, sizeof(double));
    
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

    robot_model_acados_sim_update_params(capsule, p, np);
    free(p);


    /* initialize input */
    // x
    double x0[4];
    for (int ii = 0; ii < 4; ii++)
        x0[ii] = 0.0;

    sim_in_set(robot_model_sim_config, robot_model_sim_dims,
               robot_model_sim_in, "x", x0);


    // u
    double u0[2];
    for (int ii = 0; ii < 2; ii++)
        u0[ii] = 0.0;

    sim_in_set(robot_model_sim_config, robot_model_sim_dims,
               robot_model_sim_in, "u", u0);

    // S_forw
    double S_forw[24];
    for (int ii = 0; ii < 24; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 4; ii++)
        S_forw[ii + ii * 4 ] = 1.0;


    sim_in_set(robot_model_sim_config, robot_model_sim_dims,
               robot_model_sim_in, "S_forw", S_forw);

    int status = sim_precompute(robot_model_sim_solver, robot_model_sim_in, robot_model_sim_out);

    return status;
}


int robot_model_acados_sim_solve(robot_model_sim_solver_capsule *capsule)
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(capsule->acados_sim_solver,
                           capsule->acados_sim_in, capsule->acados_sim_out);
    if (status != 0)
        printf("error in robot_model_acados_sim_solve()! Exiting.\n");

    return status;
}


int robot_model_acados_sim_free(robot_model_sim_solver_capsule *capsule)
{
    // free memory
    sim_solver_destroy(capsule->acados_sim_solver);
    sim_in_destroy(capsule->acados_sim_in);
    sim_out_destroy(capsule->acados_sim_out);
    sim_opts_destroy(capsule->acados_sim_opts);
    sim_dims_destroy(capsule->acados_sim_dims);
    sim_config_destroy(capsule->acados_sim_config);

    // free external function
    external_function_param_casadi_free(capsule->sim_forw_vde_casadi);
    external_function_param_casadi_free(capsule->sim_vde_adj_casadi);
    external_function_param_casadi_free(capsule->sim_expl_ode_fun_casadi);
    free(capsule->sim_forw_vde_casadi);
    free(capsule->sim_vde_adj_casadi);
    free(capsule->sim_expl_ode_fun_casadi);

    return 0;
}


int robot_model_acados_sim_update_params(robot_model_sim_solver_capsule *capsule, double *p, int np)
{
    int status = 0;
    int casadi_np = ROBOT_MODEL_NP;

    if (casadi_np != np) {
        printf("robot_model_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    capsule->sim_forw_vde_casadi[0].set_param(capsule->sim_forw_vde_casadi, p);
    capsule->sim_vde_adj_casadi[0].set_param(capsule->sim_vde_adj_casadi, p);
    capsule->sim_expl_ode_fun_casadi[0].set_param(capsule->sim_expl_ode_fun_casadi, p);

    return status;
}

/* getters pointers to C objects*/
sim_config * robot_model_acados_get_sim_config(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_config;
};

sim_in * robot_model_acados_get_sim_in(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_in;
};

sim_out * robot_model_acados_get_sim_out(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_out;
};

void * robot_model_acados_get_sim_dims(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_dims;
};

sim_opts * robot_model_acados_get_sim_opts(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_opts;
};

sim_solver  * robot_model_acados_get_sim_solver(robot_model_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_solver;
};

