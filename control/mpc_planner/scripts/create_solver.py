from acados_template import AcadosOcp, AcadosOcpSolver
from robot_model import robot_model
import numpy as np
import time
import math
import yaml


def Solver():
    with open("../config_params/solver_parameters.yaml", "r") as paramFile:
              params = yaml.safe_load(paramFile)
    # acados OCP handle
    model = robot_model()
    ocp = AcadosOcp()
    N = params["N_nodes"]
    r_obst = params["r_obst"]

    ocp.dims.N = N
    # OCP dimensions
    nx = 4
    nu = 2
    ny = nx + nu
    n_obst = 40
    n_o = n_obst #1 #
    # OCP costs
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = model.cost_y_expr
    ocp.model.cost_y_expr_e = model.cost_y_expr_e

    w_x = 0.25
    w_y = 0.25
    w_v = 0.0001
    w_theta = 0.001

    w_xe = 1
    w_ye = 1
    w_ve = 0.001
    w_thetae = 1
    w_a = 0.001
    w_w = 0.001

    # State and input cost
    W_obst = np.zeros(n_o)
    for i in range(n_o):
        W_obst[i]=  12


    W_x = np.array([w_x, w_y, w_v, w_theta, w_a, w_w])
    W = np.diag(np.concatenate([W_x , W_obst]))
    ocp.cost.W = W
    W_xe = np.array([w_xe,w_ye,w_ve,w_thetae])
    W_e = np.diag(np.concatenate([W_xe , W_obst]))
    ocp.cost.W_e = W_e   
    ocp.cost.yref = np.zeros([ny+n_o])
    ocp.cost.yref_e = np.zeros([nx+n_o])


    x_max = 200
    x_min = -200
    y_max = 200
    y_min = -200
    v_max = 2 #params["v_max"] 
    v_min = 0
    theta_max = 100
    theta_min = -100
    a_max = 0.1 # params["a_max"]
    a_min = -0.1 #params["a_min"]
    w_max = 0.6 #params["w_max"]
    w_min = -0.6 #params["w_min"]

    
    ocp.constraints.idxbx = np.array([0, 1, 2 , 3])
    ocp.constraints.lbx = np.array([x_min, y_min, v_min , theta_min])
    ocp.constraints.ubx = np.array([x_max, y_max, v_max , theta_max])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([a_min, w_min ])
    ocp.constraints.ubu = np.array([a_max, w_max ])

 #   num_edge = 4
 #   paramters = 100*np.ones(4*num_edge) # np.concatenate([paramters_static,paramters_dynami])# , parameters_seman,parameters_gress])

    paramters_static = [100 , 100] * n_obst
    paramters = np.concatenate([paramters_static])

    ocp.parameter_values = paramters

                                
    x0 = np.array([0, 0, 0 , 0])
    ocp.constraints.x0 = x0

    ocp.model = model

    ocp.solver_options.tf = 3
    ocp.solver_options.qp_solver =  'PARTIAL_CONDENSING_HPIPM'  #PARTIAL_CONDENSING_HPIPM
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.levenberg_marquardt = 1e-3
    ocp.solver_options.nlp_solver_max_iter = 40
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.nlp_solver_tol_stat = 1e-2
    ocp.solver_options.nlp_solver_tol_eq = 1e-2
    ocp.solver_options.print_level = 0
 #   ocp.solver_options.warm_start = 2

    acados_solver = AcadosOcpSolver(ocp, json_file="acados_solver.json")

    return acados_solver


Solver()
print("Acados solver for NMPC problem was generated")