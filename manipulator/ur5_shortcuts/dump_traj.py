#!/usr/bin/python3
import rtde_control
import rtde_receive

UR_IP = "192.168.131.40"

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP )
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
rtde_c.teachMode()

trajectory = []
while True:
    print('press enter to write point, "e" for exit')
    inp = input()
    if inp == 'e':
        break
    point = rtde_r.getActualQ()
    trajectory.append(point)
    print(point)
    
import pickle
with open('trajectory.pkl', 'wb') as fd:
    pickle.dump(trajectory, fd)