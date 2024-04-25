from robot_model import robot_model
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
import yaml
import create_solver
from math import pi, cos, sin , sqrt , exp , atan

def test_solver():

    factor = 1

    acados_solver = create_solver.Solver()
    with open("../config_params/solver_parameters.yaml", "r") as paramFile:
              params = yaml.safe_load(paramFile)
    nx = 4
    nu = 2
    ny = nx + nu
    n_obst = 40
    n_o = n_obst
    N = params["N_nodes"]

    yref = np.zeros([N,ny+n_o])
#    x_edge = np.array([ 15 , 20 , 20 , 20 ,  20.87 , 20.87,    20.87, 25])*factor   # 22.7 , 22.2,
#    y_edge = np.array([ 40 , 40,  41 , 40,   40 , 41 ,         40 , 40  ])*factor  #43.5, 42.7 ,

    x_edge = np.array([ 15 , 20 , 20 , 20 ,  20.88 , 20.88,    20.88, 25])*factor   # 22.7 , 22.2,
    y_edge = np.array([ 40 , 40,  40 , 41 ,   41 , 40 ,         40 , 40])*factor  #43.5, 42.7 ,


    x_ref_points = np.array([19 , 20.17 , 20.16 , 19 ])
    y_ref_points = np.array([39 , 39.1 , 40.4 , 40.8])

  #  x_ref_points = np.array([2, 2.5, 2.5])
  #  y_ref_points = np.array([2, 2.2, 7])
    theta_0 = 0
    theta_e = 3.14
    v_0 = 0
    v_e = 0

    x_ref = []
    y_ref = []
    theta = []
    theta_ref = []
    init_x = []
    init_y = []
    init_theta = []
    N = params["N_nodes"]
    len_segments = []
    theta = np.append(theta , theta_0 ) # current orientation robot
    theta_ref = np.append(theta_ref , theta_0 )
    num_segment = len(x_ref_points)-1
    length_path = 0
    for i in range(num_segment):
        length_path = length_path + math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2)
        theta = np.append(theta , math.atan2(y_ref_points[i+1]-y_ref_points[i], x_ref_points[i+1]-x_ref_points[i]))
        len_segments = np.append(len_segments , math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2))

    step_line = length_path / N

    print("length path",length_path)

    v_max = 0.2

    new_time_step = ((length_path)/(v_max*N)) * np.ones(N)

    print(new_time_step)

    k = 0
    x_ref = np.append(x_ref , x_ref_points[0])
    y_ref = np.append(y_ref , y_ref_points[0])
    for i in range(N+1):
        x_ref = np.append(x_ref , x_ref[i] + step_line * math.cos(theta[k+1]))
        y_ref = np.append(y_ref , y_ref[i] + step_line * math.sin(theta[k+1]))
        theta_ref = np.append(theta_ref , theta[k+1])
        d = math.sqrt((x_ref[-1]-x_ref_points[k])**2+(y_ref[-1]-y_ref_points[k])**2)
        if(d>len_segments[k] and k<(num_segment-1)):
            k = k+1
            x_ref[i] = x_ref_points[k]
            y_ref[i] = y_ref_points[k]
        elif (k>(num_segment-1)):
            break
    x0 = np.array([x_ref_points[0],y_ref_points[0],v_0,theta_0])

    init_x = x_ref[0:N+1]
    init_y = y_ref[0:N+1]
    init_theta = theta_ref[0:N+1]
    x_goal = np.array([init_x[-1],init_y[-1], v_e,theta_e])

    

    # num_edge = 4
    # parameter_values = 100*np.ones(4*num_edge)
    # parameter_values[0:len(x_edge)] = x_edge
    # parameter_values[int(num_edge*2):len(y_edge)+ int(num_edge*2)] = y_edge


    paramters_static = [100 , 100] * n_obst

   # x_edge = np.array([20, 5 , 5  , 20,         20.3,20.9 ,   24.3, 23.3 ])*factor   # 22.7 , 22.2,
   # y_edge = np.array([35, 13 , 20 , 42,        41.3 , 42 ,   47.6, 47.6 ])*factor  #43.5, 42.7 ,

    # obst_x = []
    # obst_y = []

    # obst1_x = np.linspace(16,20,10)
    # obst1_y = np.linspace(40,40,10)

    # obst2_x = np.linspace(20.87,25,8)
    # obst2_y = np.linspace(40,40,8)

    # obst_x = np.append(obst1_x , obst2_x)
    # obst_y = np.append(obst1_y , obst2_y)

    # h = 0
    # for i in range(18):
    #   paramters_static[h] = obst_x[i]
    #   paramters_static[h+1] = obst_y[i]
    #   h=h+3

    paramters_static[0] = 20
    paramters_static[1] = 40
    paramters_static[2] = 20.88
    paramters_static[3] = 40
    paramters_static[4] = 19.5
    paramters_static[5] = 40
    # paramters_static[6] = 21.5
    # paramters_static[7] = 40
    # paramters_static[8] = 18
    # paramters_static[9] = 39
    # paramters_static[10] = 18.5
    # paramters_static[11] = 40
    # paramters_static[12] = 18.2
    # paramters_static[13] = 40
    parameter_values = np.concatenate([paramters_static])

    yref[:,0]=init_x[0:N]
    yref[:,1]=init_y[0:N]
    yref[:,2] = 0.2
    yref[:,3] = init_theta[0:N]

    a = np.zeros(n_o)
    yref_e = np.concatenate([x_goal,a])  
    x_traj_init = np.transpose([ yref[:,0] , yref[:,1] , yref[:,2], yref[:,3]])


    simX = np.zeros((N+1, nx))
    simU = np.zeros((N, nu))

    #acados_solver.constraints.x0 = x0
    for i in range(N):
        acados_solver.set(i,'p',parameter_values)
        acados_solver.set(i,'y_ref',yref[i])
        acados_solver.set(i, 'x', x_traj_init[i])
        acados_solver.set(i, 'u', np.array([v_max, 0.0]))
    acados_solver.set(N, 'p',  parameter_values)
    acados_solver.set(N, 'y_ref', yref_e)
    acados_solver.set(N, 'x', x_goal)
    acados_solver.set(0,'lbx', x0)
    acados_solver.set(0,'ubx', x0)
    acados_solver.options_set('rti_phase', 0)
    acados_solver.set_new_time_steps(new_time_step)

    t = time.time()
   
    status = acados_solver.solve()
    ROB_x = np.zeros([N+1,5])
    ROB_y = np.zeros([N+1,5])
    elapsed = 1000 * (time.time() - t)
    for i in range(N + 1):
        x = acados_solver.get(i, "x")
        
        print(i , x[0],x[1],x[2],x[3])
        simX[i,:]=x
        ROB_x[i,0] = simX[i,0] + 0.6 * cos(simX[i,3]-0.59)
        ROB_x[i,1] = simX[i,0] + 0.6 * cos(simX[i,3]+0.59)
        ROB_x[i,2] = simX[i,0] - 0.6 * cos(simX[i,3]-0.59)
        ROB_x[i,3] = simX[i,0] - 0.6 * cos(simX[i,3]+0.59)
        ROB_x[i,4] = simX[i,0] + 0.6 * cos(simX[i,3]-0.59)
        ROB_y[i,0] = simX[i,1] + 0.6 * sin(simX[i,3]-0.59)
        ROB_y[i,1] = simX[i,1] + 0.6 * sin(simX[i,3]+0.59)
        ROB_y[i,2] = simX[i,1] - 0.6 * sin(simX[i,3]-0.59)
        ROB_y[i,3] = simX[i,1] - 0.6 * sin(simX[i,3]+0.59)
        ROB_y[i,4] = simX[i,1] + 0.6 * sin(simX[i,3]-0.59)
        
    for i in range(N):
        u = acados_solver.get(i, "u")
        simU[i,:]=u
        print(i , u[0],u[1])
    print("status" , status)
    cost = acados_solver.get_cost()
    print("Elapsed time: {} ms".format(elapsed))
    print("cost", cost)

    cost_cord = 0
    cost_obst = 0
    w_x = 5
    w_y = 5
    w_theta = 2
    w_xe = 5
    w_ye = 5
    w_v = 0.1


    obst = 0
    for i in range(0,N):
        cost_cord = cost_cord + (((yref[i,0]-simX[i,0])**2)*w_x + ((yref[i,1]-simX[i,1])**2)*w_y+ ((yref[i,2]-simX[i,2])**2)*w_v + ((yref[i,3]-simX[i,3])**2)*w_theta)*new_time_step[0]

    cost_cord = cost_cord + ((x0[0]-simX[0,0])**2)*w_xe + ((x0[1]-simX[0,1])**2)*w_ye + ((x0[2]-simX[0,2])**2)*w_v + ((x0[3]-simX[0,3])**2)*w_theta
    print("cost_cord" , 0.5*cost_cord)

      
    w_i = 10
    e = 6
    print("time step" , new_time_step[0])
        

    for k in range(0,4,2):
        for i in range(N+1):
            distance = ((paramters_static[k]-simX[i,0])*cos(simX[i,3]) + (paramters_static[k+1]-simX[i,1])*sin(simX[i,3]))**e / (0.5)**e +\
                       (-(paramters_static[k]-simX[i,0])*sin(simX[i,3]) + (paramters_static[k+1]-simX[i,1])*cos(simX[i,3]))**e / (0.36)**e 
            obst = (pi/2 + atan( w_i - distance*w_i))
            cost_obst = cost_obst + (obst**2)*new_time_step[0]

        
    print("cost_obst", 0.5*obst)


    print("cal cost " , 0.5*cost_cord + 0.5*obst)


    
    fig, ax1 = plt.subplots(1)

    for i in range(0,4,2):
       circle1 = plt.Circle((paramters_static[i], paramters_static[i+1]),  0.34, color='r')

       ax1.add_patch(circle1)
    
    for i in range(N):
       ax1.plot([ROB_x[i,0],ROB_x[i,1],ROB_x[i,2],ROB_x[i,3],ROB_x[i,0]], [ROB_y[i,0],ROB_y[i,1],ROB_y[i,2],ROB_y[i,3],ROB_y[i,0]], color='k')
    

    for i in range(0,len(x_edge),2):
        ax1.plot([x_edge[i] , x_edge[i+1]], [y_edge[i] , y_edge[i+1]], linewidth=2, color='g')
    
 #   ax1.plot([10,50],[0,20])
    ax1.plot(simX[:, 0], simX[:, 1] , linewidth=4 , marker='o')
    ax1.plot(init_x , init_y, marker='o')

   # plt.grid()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()

    tt = np.linspace(0,20,N+1)
    fig, ax2 = plt.subplots(4)
    ax2[0].plot(tt, simX[:,0])
    ax2[0].grid()
    ax2[1].grid()
    ax2[1].plot(tt, simX[:,1])
    ax2[2].plot(tt, simX[:,3])
    ax2[2].grid()
    ax2[3].plot(tt, simX[:,2])
    ax2[3].grid()
   # ax2[3].set_ylim([0, 1.1])
    plt.setp(ax2[0], ylabel='x (m)')
    plt.setp(ax2[1], ylabel='y (m)')
    plt.setp(ax2[2], ylabel='theta (rad)')
    plt.setp(ax2[3], ylabel='v (m/sec)')

    
    

    return

    


test_solver()

