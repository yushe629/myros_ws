#!/usr/bin/python
import math
import rospy
import numpy as np

max_val = 100
gas_origin = [2.0, 0.5, 0.0]
time_rate = 1.0
time_rate_off = 1.0
start_time = 0

def calc_gas(pos):
    dist = np.linalg.norm(gas_origin - pos)
    del_time = rospy.get_time() - start_time
    val = max_val*math.exp(-dist**2)*math.exp(-time_rate/(del_time + time_rate_off))
    return val

if __name__ == "__main__":
    calc_gas([0,0,0])