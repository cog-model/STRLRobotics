from acados_template import AcadosModel
#from casadi import SX, vertcat, sin, cos, tan, exp, if_else, pi , atan , logic_and , sqrt 
import casadi as cd
import numpy as np
import yaml

def robot_model():
    with open("../config_params/solver_parameters.yaml", "r") as paramFile:
              params = yaml.safe_load(paramFile)
    r_obst = params["r_obst"]
    model_name = "robot_model"

    # State
    x = cd.SX.sym('x') 
    y = cd.SX.sym('y')   
    v = cd.SX.sym('v')  
    theta = cd.SX.sym('theta') 
    sym_x = cd.vertcat(x, y, v ,theta)

    # Input
    a = cd.SX.sym('a')
    w = cd.SX.sym('w')
    
    sym_u = cd.vertcat(a, w)

    # Derivative of the States
    x_dot = cd.SX.sym('x_dot')
    y_dot = cd.SX.sym('y_dot')
    theta_dot = cd.SX.sym('theta_dot')
    v_dot = cd.SX.sym('v_dot')
    x_dot = cd.vertcat(x_dot, y_dot, v_dot, theta_dot)

    ## Model of Robot
    f_expl = cd.vertcat(sym_x[2] * cd.cos(sym_x[3]),
                    sym_x[2] * cd.sin(sym_x[3]),
                    sym_u[0],
                    sym_u[1])
    f_impl = x_dot - f_expl

    model = AcadosModel()



    num_static_obst = 40
    size_parameter_static = 2
    obst_param = cd.SX.sym('p', size_parameter_static * num_static_obst)
    sym_p = cd.vertcat(obst_param)

    value_stat = obst_param[0:size_parameter_static*num_static_obst]

    x_stat = value_stat[0::size_parameter_static]
    y_stat = value_stat[1::size_parameter_static]

    obst_stat = cd.SX.sym('obst_stat',num_static_obst)
    L1 = 0.6
    L = 0.285
    r_obst = 0.36
    e = 6
    # cost function for a static obstacles
    for i in range(num_static_obst): 

        ### first case
        # sigma = 0.4
        # distance0 = (x -x_stat[i])**2/r_obst**2 + (y -y_stat[i])**2/r_obst**2
        # distance1 = (x + L * cd.cos(theta)-x_stat[i])**2/r_obst**2 + (y + L * cd.sin(theta) - y_stat[i])**2/r_obst**2
        # distance2 = (x - L * cd.cos(theta)-x_stat[i])**2/r_obst**2 + (y - L * cd.sin(theta) - y_stat[i])**2/r_obst**2       
  
        # obst_stat[i] = 100*( 1/(sigma*cd.sqrt(2*cd.pi))*cd.exp(-(distance0)**2/(2*sigma**2)) +\
        #                      1/(sigma*cd.sqrt(2*cd.pi))*cd.exp(-(distance1)**2/(2*sigma**2)) +\
        #                      1/(sigma*cd.sqrt(2*cd.pi))*cd.exp(-(distance2)**2/(2*sigma**2))) 
        

        distance = ((x_stat[i]-x)*cd.cos(theta) + (y_stat[i]-y)*cd.sin(theta))**e / (0.55)**e +\
                    (-(x_stat[i]-x)*cd.sin(theta) + (y_stat[i]-y)*cd.cos(theta))**e / (0.38)**e


        ## second case
        sigma = 0.8
        
        obst_stat[i] = 3*( 1/(sigma*cd.sqrt(2*cd.pi))*cd.exp(-(distance)**2/(2*sigma**2)))


        ## third case      w_x = 15   w_y = 15   w_theta = 5   w_v = 3 W_xe = np.array([25,25,15,15])
    
   #    w_i = 10
   #    obst_stat[i] = 0.9*(cd.pi/2 + cd.atan( w_i - distance*w_i))

        ## fourth case
        # distance0 = (x -x_stat[i])**2/r_obst**2 + (y -y_stat[i])**2/r_obst**2
        # distance1 = (x + L * cd.cos(theta)-x_stat[i])**2/r_obst**2 + (y + L * cd.sin(theta) - y_stat[i])**2/r_obst**2
        # distance2 = (x - L * cd.cos(theta)-x_stat[i])**2/r_obst**2 + (y - L * cd.sin(theta) - y_stat[i])**2/r_obst**2  
        # w_i = 20
        # w_o = 0.6
        # obst_stat[i] = w_o*(cd.pi/2 + cd.atan( w_i - distance0*w_i)+\
        #                     cd.pi/2 + cd.atan( w_i - distance1*w_i)+\
        #                         cd.pi/2 + cd.atan( w_i - distance2*w_i))

        



    #obst_all = 0
    #for i in range(num_static_obst):
    #    obst_all = obst_all + obst_stat[i]
    #print(obst_all)
    
    model.cost_y_expr = cd.vertcat(sym_x, sym_u, obst_stat )
    model.cost_y_expr_e = cd.vertcat(sym_x, obst_stat )  
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = x_dot
    model.u = sym_u
    model.p = sym_p
    model.name = model_name
    print(model.cost_y_expr)

    return model