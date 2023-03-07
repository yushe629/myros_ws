#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

# read from bag file
bag = rosbag.Bag(filename)
tvoc_data2=None
tvoc_data3=None
tvoc_data4=None
pwm_data = None
for topic, msg, t in bag.read_messages():
    if topic=="/m5stack_2/tvoc":
        sensor2_data=np.array([[0.0, 0.0, 0.0]])
        sensor2_data[0,0]=msg.data
        sensor2_data[0,1]=t.secs
        sensor2_data[0,2]=t.nsecs
        if tvoc_data2 is None:
            tvoc_data2=sensor2_data
        else:
            tvoc_data2=np.append(tvoc_data2,sensor2_data,axis=0)
    elif topic=="/m5stack_3/tvoc":
        sensor3_data=np.array([[0.0, 0.0, 0.0]])
        sensor3_data[0,0]=msg.data
        sensor3_data[0,1]=t.secs
        sensor3_data[0,2]=t.nsecs
        if tvoc_data3 is None:
            tvoc_data3=sensor3_data
        else:
            tvoc_data3=np.append(tvoc_data3,sensor3_data,axis=0)
    elif topic=="/m5stack_4/tvoc":
        sensor4_data=np.array([[0.0, 0.0, 0.0]])
        sensor4_data[0,0]=msg.data
        sensor4_data[0,1]=t.secs
        sensor4_data[0,2]=t.nsecs
        if tvoc_data4 is None:
            tvoc_data4=sensor4_data
        else:
            tvoc_data4=np.append(tvoc_data4,sensor4_data,axis=0)
    elif topic=="/pwm_test":
        pwm_datum=np.array([[0.0, 0.0, 0.0]])
        pwm_datum[0,0]=msg.data
        pwm_datum[0,1]=t.secs
        pwm_datum[0,2]=t.nsecs
        if pwm_data is None:
            pwm_data=pwm_datum
        else:
            pwm_data=np.append(pwm_data,pwm_datum,axis=0)
print([pwm_data[0,1],pwm_data[0,2]])

dataset = [tvoc_data2, tvoc_data3, tvoc_data4 ,pwm_data]

# # reform time
# start_sec=np_poses[0,3]
# start_nsec=np_poses[0,4]
# t=np.zeros(np_poses.shape[0],dtype='float32')
# for i in range(np_poses.shape[0]):
#     t[i]=(np_poses[i,3]-start_sec)+(np_poses[i,4]-start_nsec)/1000000000.0

# reform time
start_sec= min([tvoc_data2[0,1], tvoc_data3[0,1], tvoc_data4[0,1]])
start_nsec= min([tvoc_data2[0,2], tvoc_data3[0,2], tvoc_data4[0,2]])
print(start_sec)
print(start_nsec)

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

plt.title("sensor value")
plt.plot(toRawTime(tvoc_data2[:,1], tvoc_data2[:,2]), tvoc_data2[:,0], 'r', label='sensor2')
plt.plot(toRawTime(tvoc_data3[:,1], tvoc_data3[:,2]), tvoc_data3[:,0], 'b', label='sensor3')
plt.plot(toRawTime(tvoc_data4[:,1], tvoc_data4[:,2]), tvoc_data4[:,0], 'g', label='sensor4')
# plt.vlines(pwm_data[0,1], 65000, -5000, olor='y', label='spin')
plt.axvline(pwm_data[0,1], color='k', lineStyle='dotted')
plt.xlabel("times[s]")
plt.ylabel("tvoc_value[ppd]")
plt.legend()

plt.show()


bag.close()
