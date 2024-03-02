import rosbag
import rospy
import bagpy
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import math
from transformations import euler_from_quaternion
from allantools import oadev
import csv

# from gps_driver.msg import Customgps

#function to take whats needed from the data
def process_imu_data(file):
    #data that will be returned
    yaw = []
    pitch = []
    roll = []
    acc_x = []
    acc_y = []
    acc_z = []

    time = []

    with rosbag.Bag(file) as bag:
        pts = 0
        t_first = 0.0

          #stepping thru each line in the bag file and extracting data needed
        for topic, msg, t in bag.read_messages(topics = "/vectornav"):
            vnRead = msg.data
            vnSplit = vnRead.split(",")
            # print(vnSplit)

            try:
                yaw.append(float(vnSplit[1]))
                pitch.append(float(vnSplit[2]))
                roll.append(float(vnSplit[3]))
                acc_x.append(float(vnSplit[7]))
                acc_y.append(float(vnSplit[8]))
                acc_z.append(float(vnSplit[9]))

                if pts == 0:
                    t_first = t
                    t_first = t_first.to_sec()
                
                #subtracting first value of time to normalize
                time_now = t
                time_now = time_now.to_sec()
                time_now = time_now - t_first

                time.append(time_now)
                pts += 1

            except ValueError:
                pass

    time = np.array(time)

    return[np.array(yaw), np.array(pitch), np.array(roll), np.array(acc_x), np.array(acc_y), np.array(acc_z), time]


#processing data from vectornav bag file
data_5hr_path = '../data/LocationC.bag'

yaw, pitch, roll, acc_x, acc_y, acc_z, time = process_imu_data(data_5hr_path)

print("imu data finished processing")

#allan variance & noise parameters
taus_y, ad_y, *_ = oadev(yaw, rate = 40.0, data_type = 'freq', taus="all")
print("yaw allan variance finished")
# taus_p, ad_p, ade_p, ns_p = oadev(pitch, rate = 40.0, data_type = 'freq', taus="all")
# print("pitch allan variance finished")
# taus_r, ad_r, ade_r, ns_r = oadev(roll, rate = 40.0, data_type = 'freq', taus="all")
# print("roll allan variance finished")
taus_ax, ad_ax, ade_ax, ns_ax = oadev(acc_x, rate = 40.0, data_type = 'freq', taus="all")
print("acc_x allan variance finished")
# taus_ay, ad_ay, ade_ay, ns_ay = oadev(acc_y, rate = 40.0, data_type = 'freq', taus="all")
# print("acc_y allan variance finished")
# taus_az, ad_az, ade_az, ns_az = oadev(acc_z, rate = 40.0, data_type = 'freq', taus="all")
# print("acc_z allan variance finished")

# print("allan variance parameters finished processing")

# data = [taus_y, ad_y, taus_p, ad_p, taus_r, ad_r, taus_ax, ad_ax, taus_ay, ad_ay, taus_az, ad_az]

# with open('5hr.csv','w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerows(data)

data = [taus_y, ad_y]

with open('5hr.csv','w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data)

print("backup allan variance parameters written to csv file")

print("NOISE PARAMETERS:")

#bias instability
yb_index = np.argmin(ad_y)
# pb_index = np.argmin(ad_p)
# rb_index = np.argmin(ad_r)
axb_index = np.argmin(ad_ax)
# ayb_index = np.argmin(ad_ay)
# azb_index = np.argmin(ad_az)

yb_min = ad_y[yb_index]
print("Yaw bias instability is " ,yb_min)
# pb_min = ad_p[pb_index]
# print("Pitch bias instability is " ,pb_min)
# rb_min = ad_r[rb_index]
# print("Roll bias instability is " ,rb_min)
axb_min = ad_ax[axb_index]
print("Acc_x bias instability is " ,axb_min)
# ayb_min = ad_ay[ayb_index]
# print("Acc_y bias instability is " ,ayb_min)
# azb_min = ad_az[azb_index]
# print("Acc_z bias instability is " ,azb_min)

#angle random walk
y_awr_index = np.where(taus_y == 1)[0][0]
y_awr = ad_y[y_awr_index]
print("Yaw AWR is ", y_awr)
# p_awr_index = taus_p.index(1)
# p_awr = ad_p[p_awr_index]
# print("Pitch AWR is ", p_awr)
# r_awr_index = taus_r.index(1)
# r_awr = ad_r[r_awr_index]
# print("Roll AWR is ", r_awr)
ax_awr_index = np.where(taus_ax == 1)[0][0]
ax_awr = ad_ax[ax_awr_index]
print("Acc_x AWR is ", ax_awr)
# ay_awr_index = taus_ay.index(1)
# ay_awr = ad_ay[ay_awr_index]
# print("Acc_y AWR is ", ay_awr)
# az_awr_index = taus_az.index(1)
# az_awr = ad_az[az_awr_index]
# print("Acc_z AWR is ", az_awr)


print("PLOTTING...")

plt.figure()
plt.loglog(taus_y, ad_y, color='red', linestyle='-', label='yaw')
# plt.loglog(taus_p, ad_p, color='green', linestyle='-', label='pitch')
# plt.loglog(taus_r, ad_r, color='blue', linestyle='-', label='roll')
plt.xlabel(r'$\tau$ (sec)')
plt.ylabel('Allan Deviation ($\sigma^2$)')
plt.legend()

plt.figure()
plt.loglog(taus_ax, ad_ax, color='red', linestyle='-', label='X-axis')
# plt.loglog(taus_ay, ad_ay, color='green', linestyle='-', label='Y-axis')
# plt.loglog(taus_az, ad_az, color='blue', linestyle='-', label='Z-axis')
plt.xlabel(r'$\tau$ (sec)')
plt.ylabel('Allan Deviation ($\sigma^2$)')
plt.legend()

plt.show()



