import rosbag
import rospy
import bagpy
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import math
from transformations import euler_from_quaternion

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
    rot_x = []
    rot_y = []
    rot_z = []

    time = []

    with rosbag.Bag(file) as bag:
        pts = 0
        t_first = 0.0

        #stepping thru each line in the bag file and extracting data needed
        for topic, msg, t in bag.read_messages(topics = "/imu"):
            quaternion = (msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w)

            r, p, y = euler_from_quaternion(quaternion)

            yaw.append(y*(180/math.pi))
            pitch.append(p*(180/math.pi))
            roll.append(r*(180/math.pi))
            acc_x.append(msg.imu.linear_acceleration.x)
            acc_y.append(msg.imu.linear_acceleration.y)
            acc_z.append(msg.imu.linear_acceleration.z)
            rot_x.append((msg.imu.angular_velocity.x)*(180/math.pi))
            rot_y.append((msg.imu.angular_velocity.y)*(180/math.pi))
            rot_z.append((msg.imu.angular_velocity.z)*(180/math.pi))

            if pts == 0:
                t_first = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
                t_first = t_first.to_sec()
            
            #subtracting first value of time to normalize
            time_now = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
            time_now = time_now.to_sec()
            time_now = time_now - t_first

            time.append(time_now)
            pts += 1
    
    ang_vel = np.array([np.array(rot_x), np.array(rot_y), np.array(rot_z)])
    acc = np.array([np.array(acc_x), np.array(acc_y), np.array(acc_z)])
    rot = np.array([np.array(yaw), np.array(pitch), np.array(roll)])
    time = np.array(time)*5 #emulated data with sensor emulator which outputs at 200hz so data compressed down to 2 mins, stretching back to 10 mins
    
    return[ang_vel, acc, rot, time]



#processing data from vectornav bag file
data_10min_path = '../data/vn_data_final.bag'
data_5hr_path = '../data/LocationC.bag'

ang_vel_short, acc_short, rot_short, time_short = process_imu_data(data_10min_path)
ang_vel_long, acc_long, rot_long, time_long = process_imu_data(data_5hr_path)

#10 min data plots

#angular velocity
fig3, axs3 = plt.subplots(3,figsize=(15,12))
axs3[0].scatter(time_short, ang_vel_short[0], color='red', label='X-axis')
axs3[1].scatter(time_short, ang_vel_short[1], color='blue', label='Y-axis')
axs3[2].scatter(time_short, ang_vel_short[2], color='green', label='Z-axis')
axs3[2].set_xlabel('Timestamp (sec)')
axs3[0].set_ylabel('Angular Velocity (deg/s)')
axs3[1].set_ylabel('Angular Velocity (deg/s)')
axs3[2].set_ylabel('Angular Velocity (deg/s)')
axs3[0].legend()
axs3[1].legend()
axs3[2].legend()
axs3[0].set_title('Angular Velocity vs. Time')


#linear acceleration
fig2, axs2 = plt.subplots(3,figsize=(15,12))
axs2[0].scatter(time_short, acc_short[0], color='red', label='X-axis')
axs2[1].scatter(time_short, acc_short[1], color='blue', label='Y-axis')
axs2[2].scatter(time_short, acc_short[2], color='green', label='Z-axis')
axs2[2].set_xlabel('Timestamp (sec)')
axs2[0].set_ylabel('Linear Acceleration (m/s^2)')
axs2[1].set_ylabel('Linear Acceleration (m/s^2)')
axs2[2].set_ylabel('Linear Acceleration (m/s^2)')
axs2[0].legend()
axs2[1].legend()
axs2[2].legend()
axs2[0].set_title('Linear Acceleration vs. Time')

#yaw, pitch, roll
fig1, axs1 = plt.subplots(3,figsize=(15,12))
axs1[0].scatter(time_short, rot_short[0], color='red', label='Yaw')
axs1[1].scatter(time_short, rot_short[1], color='blue', label='Pitch')
axs1[2].scatter(time_short, rot_short[2], color='green', label='Roll')
axs1[2].set_xlabel('Timestamp (sec)')
axs1[0].set_ylabel('Rotaton (deg)')
axs1[1].set_ylabel('Rotaton (deg)')
axs1[2].set_ylabel('Rotaton (deg)')
axs1[0].legend()
axs1[1].legend()
axs1[2].legend()
axs1[0].set_title('Rotation vs. Time')

#Histograms
fig, axs = plt.subplots(3,figsize=(15,12))
axs[0].hist(rot_short[0], bins = 50, color='red', label='Yaw')
axs[0].set_ylabel('Frequency')
axs[0].set_title('Rotation Histograms (deg)')
axs[0].legend()


axs[1].hist(rot_short[1], bins = 50, color='green', label='Pitch')
axs[1].set_ylabel('Frequency')
axs[1].legend()

axs[2].hist(rot_short[2], bins = 50, color='blue', label='Roll')
axs[2].set_ylabel('Frequency')
axs[2].legend()
plt.show()
