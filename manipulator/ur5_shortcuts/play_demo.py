#!/usr/bin/python3
import rtde_control
import rtde_receive
from typing import List

UR_IP = "192.168.131.40"

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP )
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
rtde_c.endTeachMode()

import pickle
with open('trajectory.pkl', 'rb') as fd:
    trajectory : List[List[float]]=pickle.load(fd)
for point in trajectory:
    rtde_c.moveJ(point, 0.7, 0.4)

for point in trajectory[::-1]:
    rtde_c.moveJ(point, 0.7, 0.4)

