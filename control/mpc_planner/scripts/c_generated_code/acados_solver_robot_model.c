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
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "robot_model_model/robot_model_model.h"
#include "robot_model_constraints/robot_model_constraints.h"
#include "robot_model_cost/robot_model_cost.h"




#include "acados_solver_robot_model.h"

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
// #define N      ROBOT_MODEL_N
#define NH     ROBOT_MODEL_NH
#define NPHI   ROBOT_MODEL_NPHI
#define NHN    ROBOT_MODEL_NHN
#define NPHIN  ROBOT_MODEL_NPHIN
#define NR     ROBOT_MODEL_NR


// ** solver data **

robot_model_solver_capsule * robot_model_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(robot_model_solver_capsule));
    robot_model_solver_capsule *capsule = (robot_model_solver_capsule *) capsule_mem;

    return capsule;
}


int robot_model_acados_free_capsule(robot_model_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int robot_model_acados_create(robot_model_solver_capsule* capsule)
{
    int N_shooting_intervals = ROBOT_MODEL_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return robot_model_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int robot_model_acados_update_time_steps(robot_model_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "robot_model_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for robot_model_acados_create: step 1
 */
void robot_model_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/
    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
}


/**
 * Internal function for robot_model_acados_create: step 2
 */
ocp_nlp_dims* robot_model_acados_create_2_create_and_set_dimensions(robot_model_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 4;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);

    for (int i = 0; i < N; i++)
    {
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

return nlp_dims;
}


/**
 * Internal function for robot_model_acados_create: step 3
 */
void robot_model_acados_create_3_create_and_set_functions(robot_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;


    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 80); \
    }while(false)




    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], robot_model_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], robot_model_expl_ode_fun);
    }


    // nonlinear least squares function
    MAP_CASADI_FNC(cost_y_0_fun, robot_model_cost_y_0_fun);
    MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, robot_model_cost_y_0_fun_jac_ut_xt);
    MAP_CASADI_FNC(cost_y_0_hess, robot_model_cost_y_0_hess);
    // nonlinear least squares cost
    capsule->cost_y_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_fun[i], robot_model_cost_y_fun);
    }

    capsule->cost_y_fun_jac_ut_xt = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], robot_model_cost_y_fun_jac_ut_xt);
    }

    capsule->cost_y_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(cost_y_hess[i], robot_model_cost_y_hess);
    }
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, robot_model_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, robot_model_cost_y_e_fun_jac_ut_xt);
    MAP_CASADI_FNC(cost_y_e_hess, robot_model_cost_y_e_hess);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for robot_model_acados_create: step 4
 */
void robot_model_acados_create_4_set_default_parameters(robot_model_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));
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

    for (int i = 0; i <= N; i++) {
        robot_model_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for robot_model_acados_create: step 5
 */
void robot_model_acados_create_5_set_nlp_in(robot_model_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps
    

    if (new_time_steps) {
        robot_model_acados_update_time_steps(capsule, N, new_time_steps);
    } else {// all time_steps are identical
        double time_step = 0.15;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);
   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 0.25;
    W_0[1+(NY0) * 1] = 0.25;
    W_0[2+(NY0) * 2] = 0.0001;
    W_0[3+(NY0) * 3] = 0.001;
    W_0[4+(NY0) * 4] = 0.001;
    W_0[5+(NY0) * 5] = 0.001;
    W_0[6+(NY0) * 6] = 12;
    W_0[7+(NY0) * 7] = 12;
    W_0[8+(NY0) * 8] = 12;
    W_0[9+(NY0) * 9] = 12;
    W_0[10+(NY0) * 10] = 12;
    W_0[11+(NY0) * 11] = 12;
    W_0[12+(NY0) * 12] = 12;
    W_0[13+(NY0) * 13] = 12;
    W_0[14+(NY0) * 14] = 12;
    W_0[15+(NY0) * 15] = 12;
    W_0[16+(NY0) * 16] = 12;
    W_0[17+(NY0) * 17] = 12;
    W_0[18+(NY0) * 18] = 12;
    W_0[19+(NY0) * 19] = 12;
    W_0[20+(NY0) * 20] = 12;
    W_0[21+(NY0) * 21] = 12;
    W_0[22+(NY0) * 22] = 12;
    W_0[23+(NY0) * 23] = 12;
    W_0[24+(NY0) * 24] = 12;
    W_0[25+(NY0) * 25] = 12;
    W_0[26+(NY0) * 26] = 12;
    W_0[27+(NY0) * 27] = 12;
    W_0[28+(NY0) * 28] = 12;
    W_0[29+(NY0) * 29] = 12;
    W_0[30+(NY0) * 30] = 12;
    W_0[31+(NY0) * 31] = 12;
    W_0[32+(NY0) * 32] = 12;
    W_0[33+(NY0) * 33] = 12;
    W_0[34+(NY0) * 34] = 12;
    W_0[35+(NY0) * 35] = 12;
    W_0[36+(NY0) * 36] = 12;
    W_0[37+(NY0) * 37] = 12;
    W_0[38+(NY0) * 38] = 12;
    W_0[39+(NY0) * 39] = 12;
    W_0[40+(NY0) * 40] = 12;
    W_0[41+(NY0) * 41] = 12;
    W_0[42+(NY0) * 42] = 12;
    W_0[43+(NY0) * 43] = 12;
    W_0[44+(NY0) * 44] = 12;
    W_0[45+(NY0) * 45] = 12;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 0.25;
    W[1+(NY) * 1] = 0.25;
    W[2+(NY) * 2] = 0.0001;
    W[3+(NY) * 3] = 0.001;
    W[4+(NY) * 4] = 0.001;
    W[5+(NY) * 5] = 0.001;
    W[6+(NY) * 6] = 12;
    W[7+(NY) * 7] = 12;
    W[8+(NY) * 8] = 12;
    W[9+(NY) * 9] = 12;
    W[10+(NY) * 10] = 12;
    W[11+(NY) * 11] = 12;
    W[12+(NY) * 12] = 12;
    W[13+(NY) * 13] = 12;
    W[14+(NY) * 14] = 12;
    W[15+(NY) * 15] = 12;
    W[16+(NY) * 16] = 12;
    W[17+(NY) * 17] = 12;
    W[18+(NY) * 18] = 12;
    W[19+(NY) * 19] = 12;
    W[20+(NY) * 20] = 12;
    W[21+(NY) * 21] = 12;
    W[22+(NY) * 22] = 12;
    W[23+(NY) * 23] = 12;
    W[24+(NY) * 24] = 12;
    W[25+(NY) * 25] = 12;
    W[26+(NY) * 26] = 12;
    W[27+(NY) * 27] = 12;
    W[28+(NY) * 28] = 12;
    W[29+(NY) * 29] = 12;
    W[30+(NY) * 30] = 12;
    W[31+(NY) * 31] = 12;
    W[32+(NY) * 32] = 12;
    W[33+(NY) * 33] = 12;
    W[34+(NY) * 34] = 12;
    W[35+(NY) * 35] = 12;
    W[36+(NY) * 36] = 12;
    W[37+(NY) * 37] = 12;
    W[38+(NY) * 38] = 12;
    W[39+(NY) * 39] = 12;
    W[40+(NY) * 40] = 12;
    W[41+(NY) * 41] = 12;
    W[42+(NY) * 42] = 12;
    W[43+(NY) * 43] = 12;
    W[44+(NY) * 44] = 12;
    W[45+(NY) * 45] = 12;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 1;
    W_e[1+(NYN) * 1] = 1;
    W_e[2+(NYN) * 2] = 0.001;
    W_e[3+(NYN) * 3] = 1;
    W_e[4+(NYN) * 4] = 12;
    W_e[5+(NYN) * 5] = 12;
    W_e[6+(NYN) * 6] = 12;
    W_e[7+(NYN) * 7] = 12;
    W_e[8+(NYN) * 8] = 12;
    W_e[9+(NYN) * 9] = 12;
    W_e[10+(NYN) * 10] = 12;
    W_e[11+(NYN) * 11] = 12;
    W_e[12+(NYN) * 12] = 12;
    W_e[13+(NYN) * 13] = 12;
    W_e[14+(NYN) * 14] = 12;
    W_e[15+(NYN) * 15] = 12;
    W_e[16+(NYN) * 16] = 12;
    W_e[17+(NYN) * 17] = 12;
    W_e[18+(NYN) * 18] = 12;
    W_e[19+(NYN) * 19] = 12;
    W_e[20+(NYN) * 20] = 12;
    W_e[21+(NYN) * 21] = 12;
    W_e[22+(NYN) * 22] = 12;
    W_e[23+(NYN) * 23] = 12;
    W_e[24+(NYN) * 24] = 12;
    W_e[25+(NYN) * 25] = 12;
    W_e[26+(NYN) * 26] = 12;
    W_e[27+(NYN) * 27] = 12;
    W_e[28+(NYN) * 28] = 12;
    W_e[29+(NYN) * 29] = 12;
    W_e[30+(NYN) * 30] = 12;
    W_e[31+(NYN) * 31] = 12;
    W_e[32+(NYN) * 32] = 12;
    W_e[33+(NYN) * 33] = 12;
    W_e[34+(NYN) * 34] = 12;
    W_e[35+(NYN) * 35] = 12;
    W_e[36+(NYN) * 36] = 12;
    W_e[37+(NYN) * 37] = 12;
    W_e[38+(NYN) * 38] = 12;
    W_e[39+(NYN) * 39] = 12;
    W_e[40+(NYN) * 40] = 12;
    W_e[41+(NYN) * 41] = 12;
    W_e[42+(NYN) * 42] = 12;
    W_e[43+(NYN) * 43] = 12;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "nls_y_hess", &capsule->cost_y_0_hess);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "nls_y_hess", &capsule->cost_y_hess[i-1]);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "nls_y_hess", &capsule->cost_y_e_hess);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(4 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -0.1;
    ubu[0] = 0.1;
    lbu[1] = -0.6;
    ubu[1] = 0.6;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);








    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 0;
    idxbx[1] = 1;
    idxbx[2] = 2;
    idxbx[3] = 3;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -200;
    ubx[0] = 200;
    lbx[1] = -200;
    ubx[1] = 200;
    ubx[2] = 2;
    lbx[3] = -100;
    ubx[3] = 100;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);







    /* terminal constraints */















}


/**
 * Internal function for robot_model_acados_create: step 6
 */
void robot_model_acados_create_6_set_opts(robot_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);


    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;

    const int qp_solver_cond_N_ori = 10;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");


    // set SQP specific options
    double nlp_solver_tol_stat = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.01;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 40;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int qp_solver_iter_max = 100;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for robot_model_acados_create: step 7
 */
void robot_model_acados_create_7_set_nlp_out(robot_model_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for robot_model_acados_create: step 8
 */
//void robot_model_acados_create_8_create_solver(robot_model_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for robot_model_acados_create: step 9
 */
int robot_model_acados_create_9_precompute(robot_model_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int robot_model_acados_create_with_discretization(robot_model_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ROBOT_MODEL_N && !new_time_steps) {
        fprintf(stderr, "robot_model_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, ROBOT_MODEL_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    robot_model_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = robot_model_acados_create_2_create_and_set_dimensions(capsule);
    robot_model_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    robot_model_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    robot_model_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    robot_model_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    robot_model_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //robot_model_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = robot_model_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int robot_model_acados_update_qp_solver_cond_N(robot_model_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from robot_model_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = robot_model_acados_create_9_precompute(capsule);
    return status;
}


int robot_model_acados_reset(robot_model_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int robot_model_acados_update_params(robot_model_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 80;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi+stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun+stage, p);
    

        // constraints
    

        // cost
        if (stage == 0)
        {
            capsule->cost_y_0_fun.set_param(&capsule->cost_y_0_fun, p);
            capsule->cost_y_0_fun_jac_ut_xt.set_param(&capsule->cost_y_0_fun_jac_ut_xt, p);
            capsule->cost_y_0_hess.set_param(&capsule->cost_y_0_hess, p);
        }
        else // 0 < stage < N
        {
            capsule->cost_y_fun[stage-1].set_param(capsule->cost_y_fun+stage-1, p);
            capsule->cost_y_fun_jac_ut_xt[stage-1].set_param(capsule->cost_y_fun_jac_ut_xt+stage-1, p);
            capsule->cost_y_hess[stage-1].set_param(capsule->cost_y_hess+stage-1, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->cost_y_e_fun.set_param(&capsule->cost_y_e_fun, p);
        capsule->cost_y_e_fun_jac_ut_xt.set_param(&capsule->cost_y_e_fun_jac_ut_xt, p);
        capsule->cost_y_e_hess.set_param(&capsule->cost_y_e_hess, p);
        // constraints
    
    }

    return solver_status;
}


int robot_model_acados_update_params_sparse(robot_model_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 80;
    if (casadi_np < n_update) {
        printf("robot_model_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("robot_model_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi+stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun+stage, n_update, idx, p);
    

        // constraints
    

        // cost
        if (stage == 0)
        {
            capsule->cost_y_0_fun.set_param_sparse(&capsule->cost_y_0_fun, n_update, idx, p);
            capsule->cost_y_0_fun_jac_ut_xt.set_param_sparse(&capsule->cost_y_0_fun_jac_ut_xt, n_update, idx, p);
            capsule->cost_y_0_hess.set_param_sparse(&capsule->cost_y_0_hess, n_update, idx, p);
        }
        else // 0 < stage < N
        {
            capsule->cost_y_fun[stage-1].set_param_sparse(capsule->cost_y_fun+stage-1, n_update, idx, p);
            capsule->cost_y_fun_jac_ut_xt[stage-1].set_param_sparse(capsule->cost_y_fun_jac_ut_xt+stage-1, n_update, idx, p);
            capsule->cost_y_hess[stage-1].set_param_sparse(capsule->cost_y_hess+stage-1, n_update, idx, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->cost_y_e_fun.set_param_sparse(&capsule->cost_y_e_fun, n_update, idx, p);
        capsule->cost_y_e_fun_jac_ut_xt.set_param_sparse(&capsule->cost_y_e_fun_jac_ut_xt, n_update, idx, p);
        capsule->cost_y_e_hess.set_param_sparse(&capsule->cost_y_e_hess, n_update, idx, p);
        // constraints
    
    }


    return solver_status;
}

int robot_model_acados_solve(robot_model_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int robot_model_acados_free(robot_model_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost
    external_function_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    external_function_param_casadi_free(&capsule->cost_y_0_hess);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
        external_function_param_casadi_free(&capsule->cost_y_hess[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    free(capsule->cost_y_hess);
    external_function_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);
    external_function_param_casadi_free(&capsule->cost_y_e_hess);

    // constraints

    return 0;
}


void robot_model_acados_print_stats(robot_model_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[480];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");

    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }

}

int robot_model_acados_custom_update(robot_model_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *robot_model_acados_get_nlp_in(robot_model_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *robot_model_acados_get_nlp_out(robot_model_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *robot_model_acados_get_sens_out(robot_model_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *robot_model_acados_get_nlp_solver(robot_model_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *robot_model_acados_get_nlp_config(robot_model_solver_capsule* capsule) { return capsule->nlp_config; }
void *robot_model_acados_get_nlp_opts(robot_model_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *robot_model_acados_get_nlp_dims(robot_model_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *robot_model_acados_get_nlp_plan(robot_model_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
