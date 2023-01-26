#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import matplotlib
del matplotlib.font_manager.weight_dict['roman']
matplotlib.font_manager._rebuild()

# create plot for up and down subexperiment

args = sys.argv
# print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

START_TIME = 1670316905
END_TIME = 1670316979

# in this area, experiment seemed to be succeeded.
# EXPLORE_START_SEC = 1674028136
# EXPLORE_START_NSEC = 98000000

# EXPLORE_START_SEC = 1674028133
# EXPLORE_START_NSEC = 297000000

EXPLORE_START_SEC = 1674028126
EXPLORE_START_NSEC = 322000000

TRUE_PEAK_SEC = 1674028180
TRUE_PEAK_NSEC = 114945800

SCAN_START_SEC = 1674028140


EXPLORE_END_SEC = 1674028196

MAX_TIME = 1670316928.392


# read from bag file
bag = rosbag.Bag(filename)

tvoc_data = None
uav_height_data = None

uav_nav_data = None

scannning_data = None

for topic, msg, t in bag.read_messages():
    if t.secs > EXPLORE_START_SEC and t.secs < EXPLORE_END_SEC:
        if topic=="/quadrotor/tvoc":
            sensor_data=np.array([[0.0, 0.0, 0.0]])
            sensor_data[0,0]=msg.data
            sensor_data[0,1]=t.secs
            sensor_data[0,2]=t.nsecs
            if tvoc_data is None:
                tvoc_data=sensor_data
            else:
                tvoc_data=np.append(tvoc_data,sensor_data,axis=0)
            if t.secs > SCAN_START_SEC:
                if scannning_data is None:
                    scannning_data = sensor_data
                else:
                    scannning_data=np.append(scannning_data,sensor_data,axis=0)
        if topic=="/quadrotor/mocap/pose":
            mocap_data=np.array([[0.0, 0.0, 0.0]])
            mocap_data[0,0]=msg.pose.position.z
            mocap_data[0,1]=t.secs
            mocap_data[0,2]=t.nsecs
            if uav_height_data is None:
                uav_height_data=mocap_data
            else:
                uav_height_data=np.append(uav_height_data,mocap_data,axis=0)
        if topic=="/quadrotor/uav/nav":
            nav_data=np.array([[0.0, 0.0, 0.0]])
            if msg.target_pos_z == 0.7:
                nav_data[0,1] = t.secs
                nav_data[0,2] = t.nsecs
            if uav_nav_data is None:
                uav_nav_data=mocap_data
            else:
                uav_nav_data=np.append(uav_nav_data,mocap_data,axis=0)

max_index = np.argmax(tvoc_data[:,0])
max_tvoc_data = tvoc_data[max_index, :]
# print(max_tvoc_data)

explore_start_data = uav_nav_data[0]

max_height_index = np.where(uav_height_data[:,1] == max_tvoc_data[1])[0][0]
max_height_data = uav_height_data[max_height_index, :]
max_height_value = max_height_data[0]

print(max_height_data)

dataset = [tvoc_data, uav_height_data]


scanned_index = np.argmax(scannning_data[:,0])
scanned_peak_data = scannning_data[scanned_index, :]

# # # reform time
# start_sec = min([tvoc_data[0,1], uav_height_data[0,2]])
# start_nsec = min([tvoc_data[0,2], uav_height_data[0,2]])


def toRawTime(sec, nsec):
    return sec + nsec/1000000000.0


def toFormatedTime(sec, nsec):
    return toRawTime(sec, nsec)- toRawTime(EXPLORE_START_SEC, EXPLORE_START_NSEC)

peak_time = toFormatedTime(max_tvoc_data[1], max_tvoc_data[2])


SMALL_SIZE = 20
MEDIUM_SIZE = 24
LARGE_SIZE = 28
LEGEND_FONTSIZE = 14


plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = SMALL_SIZE
plt.rcParams['xtick.labelsize'] = SMALL_SIZE
plt.rcParams['ytick.labelsize'] = SMALL_SIZE
plt.rcParams['axes.titlesize'] = LARGE_SIZE
plt.rcParams['axes.labelsize'] = MEDIUM_SIZE
plt.rcParams['legend.fontsize'] = 14
plt.rcParams['figure.titlesize'] = LARGE_SIZE
# plt.rcParams["font.size"] = 15
# plt.rcParams['xtick.labelsize'] = 9
# plt.rcParams['ytick.labelsize'] = 24
plt.rcParams['xtick.direction'] = 'out'
plt.rcParams['ytick.direction'] = 'out'
plt.rcParams["xtick.major.width"] = 1.0
plt.rcParams["ytick.major.width"] = 1.0
plt.rcParams["xtick.minor.width"] = 1.0
plt.rcParams["ytick.minor.width"] = 1.0
plt.rcParams["xtick.major.size"] = 10
plt.rcParams["ytick.major.size"] = 10
plt.rcParams["xtick.minor.size"] = 5
plt.rcParams["ytick.minor.size"] = 5
plt.rcParams['xtick.minor.visible'] = False
plt.rcParams['ytick.minor.visible'] = False
plt.rcParams['xtick.top'] = False
plt.rcParams['ytick.right'] = False
plt.rcParams['axes.linewidth'] = 1.0
# plt.rcParams['axes.grid'] = True
plt.rcParams["legend.fancybox"] = False
plt.rcParams["legend.framealpha"] = 1
plt.rcParams["legend.edgecolor"] = 'black'
# plt.rcParams["legend.handlelength"] = 1
# plt.rcParams["legend.labelspacing"] = 5.
# plt.rcParams["legend.handletextpad"] = 3.
# plt.rcParams["legend.markerscale"] = 2
# plt.rcParams["legend.borderaxespad"] = 0.

fig = plt.figure(figsize=(9,6), facecolor="white")
fig.subplots_adjust(hspace=0.8, wspace=0.4)


fig1 = fig.add_subplot(211)
# fig1.set_title("uav height")
fig1.plot(toFormatedTime(uav_height_data[:,1], uav_height_data[:,2]), uav_height_data[:,0], 'g')
# tmp: explore time has to be rechecked.
fig1.axvline(peak_time, color='b', lineStyle='dotted', label="whole peak")
fig1.axvline(toFormatedTime(explore_start_data[1], explore_start_data[2]), color='k', lineStyle='dotted', label="start of scanning")
fig1.axvline(toFormatedTime(scanned_peak_data[1], scanned_peak_data[2]), color='c', lineStyle='dotted', label="peak in scanning")
fig1.set_xlabel("time [s]")
fig1.set_ylabel("uav height [m]")
# fig1.legend(loc='lower right')
# fig1.legend(bbox_to_anchor=(1.35,1), loc="upper right", borderaxespad=0)
fig1.legend()

fig2 = fig.add_subplot(212)
# fig2.set_title("gas value")
fig2.plot(toFormatedTime(tvoc_data[:,1], tvoc_data[:,2]), tvoc_data[:,0], 'r')
fig2.axvline(peak_time, color='b', lineStyle='dotted', label="whole peak")
fig2.axvline(toFormatedTime(explore_start_data[1], explore_start_data[2]), color='k', lineStyle='dotted', label="start of scanning")
fig2.axvline(toFormatedTime(scanned_peak_data[1], scanned_peak_data[2]), color='c', lineStyle='dotted', label="peak in scanning")
fig2.set_xlabel("time [s]")
fig2.set_ylabel("gas value [ppd]")
# fig2.legend(loc='lower right')
fig2.legend()



# save
# plt.show()
fig.savefig('integrate_uav.eps', bbox_inches="tight", pad_inches=0.05)
fig.savefig('integrate_uav.pdf', bbox_inches="tight", pad_inches=0.05)

bag.close()
