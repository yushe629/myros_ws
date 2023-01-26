#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# create plot for up and down subexperiment

args = sys.argv
# print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

START_TIME = 1670316905
END_TIME = 1670316979

MAX_TIME = 1670316928.392


# read from bag file
bag = rosbag.Bag(filename)

tvoc_data = None
uav_height_data = None

for topic, msg, t in bag.read_messages():
    print(topic)
    
# max_index = np.argmax(tvoc_data[:,0])
# max_tvoc_data = tvoc_data[max_index, :]

# sec = max_tvoc_data[1]
# print(sec)

# max_height_data = None

# print(uav_height_data[0])

# max_height_data = np.where(uav_height_data[:,1] == sec)

# print(max_height_data)

# print(max_tvoc_data)

# dataset = [tvoc_data, uav_height_data]

# # reform time
# start_sec=np_poses[0,3]
# start_nsec=np_poses[0,4]
# t=np.zeros(np_poses.shape[0],dtype='float32')
# for i in range(np_poses.shape[0]):
#     t[i]=(np_poses[i,3]-start_sec)+(np_poses[i,4]-start_nsec)/1000000000.0

# # reform time
# start_sec= min([tvoc_data2[0,1], tvoc_data3[0,1], tvoc_data4[0,1]])
# start_nsec= min([tvoc_data2[0,2], tvoc_data3[0,2], tvoc_data4[0,2]])
# print(start_sec)
# print(start_nsec)

# for i in range(tvoc_data2):


# # plot
# plt.subplot(121)
# plt.title("time vs x,y")
# plt.plot(t, np_poses[:,0], 'r', label="x")
# plt.plot(t, np_poses[:,1], 'b', label="y")
# plt.xlabel("time[s]")
# plt.ylabel("vel[m/s]")
# plt.legend()

# plt.subplot(122)
# plt.title("x vs y")
# plt.plot(np_poses[:,0], np_poses[:,1], 'g')
# plt.show()

def toRawTime(sec, nsec):
    return sec + nsec/1000000000.0

# plt.title("sensor value")
# plt.plot(toRawTime(tvoc_data2[:,1], tvoc_data2[:,2]), tvoc_data2[:,0], 'r', label='sensor2')
# plt.plot(toRawTime(tvoc_data3[:,1], tvoc_data3[:,2]), tvoc_data3[:,0], 'b', label='sensor3')
# plt.plot(toRawTime(tvoc_data4[:,1], tvoc_data4[:,2]), tvoc_data4[:,0], 'g', label='sensor4')
# # plt.vlines(pwm_data[0,1], 65000, -5000, olor='y', label='spin')
# plt.axvline(pwm_data[0,1], color='k', lineStyle='dotted')
# plt.xlabel("times[s]")
# plt.ylabel("tvoc_value[ppd]")
# plt.legend()

# plt.show()


bag.close()
