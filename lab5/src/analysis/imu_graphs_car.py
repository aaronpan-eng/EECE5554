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
import os
from scipy.optimize import least_squares
from scipy.signal import butter, sosfilt, freqz
import scipy.integrate as integrate
from math import fabs, pi

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
    mag_x = []
    mag_y = []
    mag_z = []
    time_imu = []

    easting = []
    northing = []
    altitude = []
    time_gps = []

    with rosbag.Bag(file) as bag:
        pts = 0
        t_first = 0.0

        #stepping thru each line in the bag file and extracting data needed
        for topic, msg, t in bag.read_messages(topics = "/imu"):
            try:
                vnRead = msg.vnymr_read
                vnRead = str(vnRead)
                vnSplit = vnRead.split(",")
                y = np.radians(float(vnSplit[1]))
                p = np.radians(float(vnSplit[2]))
                r = np.radians(float(vnSplit[3]))

                if pts == 0:
                    t_first = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
                    t_first = t_first.to_sec()
                
                #subtracting first value of time to normalize
                time_now = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
                time_now = time_now.to_sec()
                time_now = time_now - t_first
                
                # quaternion = (msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w)

                # r, p, y = euler_from_quaternion(quaternion)

                yaw.append(y)
                pitch.append(p)
                roll.append(r)
                acc_x.append(msg.imu.linear_acceleration.x)
                acc_y.append(msg.imu.linear_acceleration.y)
                acc_z.append(msg.imu.linear_acceleration.z)
                rot_x.append((msg.imu.angular_velocity.x))
                rot_y.append((msg.imu.angular_velocity.y))
                rot_z.append((msg.imu.angular_velocity.z))
                mag_x.append(msg.mag_field.magnetic_field.x)
                mag_y.append(msg.mag_field.magnetic_field.y)
                mag_z.append(msg.mag_field.magnetic_field.z)

                time_imu.append(time_now)
                pts += 1
            except ValueError:
                pass
            except IndexError:
                pass
        
        pts = 0
        t_first = 0.0
        #stepping thru each line in the bag file and extracting data needed
        for topic, msg, t in bag.read_messages(topics = "/gps"):
            try:
                east_msg = float(msg.utm_easting)
                north_msg = float(msg.utm_northing)
                alt_msg = float(msg.altitude)

                if pts == 0:
                    northing_first = north_msg
                    easting_first = east_msg
                    t_first = msg.header.stamp.secs
                
                #subtracting first value of time to normalize
                time_now = msg.header.stamp.secs
                time_now = time_now - t_first

                #subtracting first value of northing and easting from dataset to normalize
                north_msg -= northing_first
                east_msg -= easting_first
                pts += 1

                northing.append(north_msg)
                easting.append(east_msg)
                altitude.append(alt_msg)
                time_gps.append(time_now)
            except ValueError:
                pass
        
    ang_vel = np.array([np.array(rot_x), np.array(rot_y), np.array(rot_z)])
    acc = np.array([np.array(acc_x), np.array(acc_y), np.array(acc_z)])
    rot = np.array([np.array(yaw), np.array(pitch), np.array(roll)])
    mag = np.array([np.array(mag_x), np.array(mag_y), np.array(mag_z)])
    time_imu = np.array(time_imu)
    northing = np.array(northing)
    easting = np.array(easting)
    altitude = np.array(altitude)
    time_gps = np.array(time_gps)
    
    return[ang_vel, acc, rot, mag, time_imu, northing, easting, altitude, time_gps]

#This function describes my mapping from measured data back to a circle
def distortion_model(X_meas, dist_params):
    x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
    y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
    X = np.array([x,y])
    return X
    
#This function finds the difference between a circle and my transformed measurement
def residual(p, X_mag, X_meas):
    return (X_mag - distortion_model(X_meas, p)).flatten()

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

def truncated_remainder(dividend, divisor):
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)

    remainder = dividend - divisor * divided_number

    return remainder

def transform_to_pipi(input_angle):
    # revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))

    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(fabs((truncated_remainder(input_angle + pi, 2 * pi))
                                      / (2 * pi))) - 1))) * pi

    output_angle = p1 - p2

    return output_angle



path_circle_driving = '../data/data_going_in_circles.bag'
path_driving = '../data/data_driving.bag'

path_list = [path_circle_driving, 'Driving in Circles', path_driving, 'Driving on Route']

p0 = None
lsq_min = None

for i in range(len(path_list)):
    if i % 2 == 0:
        shape = path_list[i+1]
        if shape == 'Driving in Circles':
            ang, acc, rot, mag, time, *_ = process_imu_data(path_list[i])
            
            # "perfect/ideal" magnetometer readings
            field_strength = 20509e-9 #The horizontal magnetic field strength in Boston is approx 20,500 nT 

            angle = np.linspace(-4*np.pi, 4*np.pi, np.size(mag[0])) #multiply pi by 4 because we looped around in a cirlce 4 times
            
            x_mag = field_strength * np.sin(angle) 
            y_mag = field_strength * np.cos(angle) 
            X_mag = np.array([x_mag, y_mag])

            X_meas = np.array([mag[0], mag[1]])

            #Least squares optimization to find model coefficients
            p0 = [0,0,0,0,0,0]
            lsq_min = least_squares(residual, p0, args=(X_mag, X_meas))

            print("least square fitting values are: ")
            print(lsq_min.x)

            X_model = distortion_model(X_meas, lsq_min.x) 

            ##### PLOT 0: MAG BEFORE AND AFTER CALIBRATION #################################################################

            #Plotting ellipse and lsq
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))  # Creating a grid of 1 row and 2 columns

            # Plot for the left subplot
            ax1.plot(X_mag[0], X_mag[1], 'g--', label = "ideal circle")
            ax1.scatter(X_meas[0], X_meas[1], color = 'orange', label="measured data", s=1)
            ax1.axis('equal')
            ax1.legend(loc = 'upper right')
            ax1.set_xlabel('X component of magnetic field (T)')
            ax1.set_ylabel('Y component of magnetic field (T)')
            ax1.set_title(shape + ': Uncalibrated Magnetic Field Plot')

            # Plot for the right subplot
            ax2.plot(X_mag[0], X_mag[1], 'g--', label = "ideal circle")  # Assuming you want the same plot for both subplots
            ax2.scatter(X_model[0], X_model[1], color = 'blue', label="calibrated data", s=1)
            ax2.axis('equal')
            ax2.legend(loc = 'upper right')
            ax2.set_xlabel('X component of magnetic field (T)')
            ax2.set_ylabel('Y component of magnetic field (T)')
            ax2.set_title(shape + ': Calibrated Magnetic Field Plot')

            plt.show(block = False)

        elif shape == 'Driving on Route':
            ang, acc, rot, mag, time_imu, northing, easting, altitude, time_gps = process_imu_data(path_list[i])

            # Measured values for magnetometer
            X_meas = np.array([mag[0], mag[1]])

            # Calibrated values (taken from driving in circles)
            X_model = distortion_model(X_meas, lsq_min.x)

            # #Plotting
            # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))  # Creating a grid of 1 row and 2 columns

            # # Plot for the left subplot
            # ax1.scatter(X_meas[0], X_meas[1], color = 'orange', label="measured data", s=1)
            # ax1.axis('equal')
            # ax1.legend(loc = 'upper right')
            # ax1.set_xlabel('X component of magnetic field (T)')
            # ax1.set_ylabel('Y component of magnetic field (T)')
            # ax1.set_title(shape + ': Uncalibrated Magnetic Field Plot')

            # # Plot for the right subplot
            # ax2.scatter(X_model[0], X_model[1], color = 'blue', label="calibrated data", s=1)
            # ax2.axis('equal')
            # ax2.legend(loc = 'upper right')
            # ax2.set_xlabel('X component of magnetic field (T)')
            # ax2.set_ylabel('Y component of magnetic field (T)')
            # ax2.set_title(shape + ': Calibrated Magnetic Field Plot')

            ##### PLOT 1: MAGNETOMETER HEADING BEFORE AND AFTER CALIBRATION ###################################

            #mag heading calibrated and uncalibrated calculation
            mag_heading_uncalibrated = np.arctan2(X_meas[1],X_meas[0])
            mag_heading_calibrated = np.arctan2(X_model[1],X_model[0])

            fig, ax = plt.subplots(figsize=(15,8))
            ax.plot(time_imu, mag_heading_uncalibrated, label="Yaw uncalibrated")
            ax.plot(time_imu, mag_heading_calibrated, label="Yaw calibrated")
            ax.legend(loc = 'upper right')
            plt.xlabel('Time (sec)')
            plt.ylabel('Magnetometer Heading (radians)')
            plt.title(shape + ': Magnetometer Heading vs. Time')
            ax.set_ylim(-math.pi,math.pi)

            ##### PLOT 2: GYRO YAW ESTIMATION V TIME ###############################################3

            #yaw integreated from gyro
            yaw_gyro = integrate.cumulative_trapezoid(ang[2], time_imu, initial = 0)

            yaw_gyro_transformed = []

            #transforming to range -pi ot pi
            for angle in yaw_gyro:
                transformed_angle = transform_to_pipi(angle)
                yaw_gyro_transformed.append(transformed_angle)

            yaw_gyro_transformed = np.array(yaw_gyro_transformed)

            #setting frist point in mag heading calibrated to zero
            mag_heading_calibrated_unwrap = np.unwrap(mag_heading_calibrated)
            first_p = mag_heading_calibrated_unwrap[0]
            mag_heading_calibrated_unwrap = mag_heading_calibrated-first_p

            #resetting range from -pi to pi
            mag_heading_calibrated_transformed = []

            for angle in mag_heading_calibrated_unwrap:
                transformed_angle = transform_to_pipi(angle)
                mag_heading_calibrated_transformed.append(transformed_angle)

            mag_heading_calibrated_transformed = np.array(mag_heading_calibrated_transformed)

            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_imu, yaw_gyro_transformed, color = 'blue', label="Yaw (integrated - gyro)")
            # ax.plot(time_imu, mag_heading_calibrated_transformed, color = 'yellow', label="Yaw (corrected - mag)")
            ax.set_ylabel('Rotation (rad)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': Rotation vs. Time')
            ax.legend(loc = 'upper right')

            ##### PLOT 3: LPF, HPF, CPF, IMU HEADING ESTIMATE ############################################################
        
            # Lowpass filter for yaw (calibrated magnetometer estimate)
            order = 1 #you can increase this to make the filter "sharper"
            sampl_freq = 40 #change to sampling frequency of your data collection
            cutoff_freq = 1 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 
            sos_lp, y_lp_mag = butter_filter(mag_heading_calibrated_transformed, cutoff_freq, sampl_freq, "lowpass", 5)


            # Highpass filter for yaw (integraded from gyro)
            sampl_freq = 40 #change to sampling frequency of your data collection
            cutoff_freq = 0.001 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 
            sos_hp, y_hp_gyro = butter_filter(yaw_gyro_transformed, cutoff_freq, sampl_freq, "highpass", 5)

            # ENDED UP NOT NEEDING HIGHPASS FILTER ON GYRO
            # AS A RESULT JUST USED LOW PASS FROM YAW (MAGNETOMETER) AND YAW (INTEGRATED FROM GYRO)

            # Complementary filter
            alpha = 0.1
            y_cp = alpha*y_lp_mag + (1-alpha)*yaw_gyro_transformed

            # adding yaw from IMU
            yaw_imu = rot[0]
            yaw_imu = np.unwrap(yaw_imu)
            first_p = yaw_imu[0]
            yaw_imu = yaw_imu - first_p

            yaw_imu_transformed = []

            for angle in yaw_imu:
                transformed_angle = transform_to_pipi(angle)
                yaw_imu_transformed.append(transformed_angle)
            
            yaw_imu_transformed = np.array(yaw_imu_transformed)

            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_imu, yaw_gyro_transformed, color = 'blue', label="Yaw (integrated gyro)")
            ax.plot(time_imu, y_hp_gyro, color = 'yellow', label="HPF: Yaw (integrated gyro) [0.001]")
            ax.plot(time_imu, mag_heading_calibrated_transformed, color = 'purple', label="Yaw (corrected mag)")
            ax.plot(time_imu, y_lp_mag, color='red', label="LPF: Yaw (corrected mag) [1]")
            ax.plot(time_imu, yaw_imu_transformed, color = 'black', label="Yaw (imu)")
            ax.plot(time_imu, y_cp, color="cyan", label="CF: LPF + Yaw (integrated gyro)")
            ax.set_ylabel('Yaw (rad)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': Yaw Filtered Data')
            ax.legend(loc = 'upper right')

            # fig, ax = plt.subplots(1, figsize=(15,8))
            # ax.plot(time_imu, yaw_gyro_transformed, color = 'blue', label="Yaw (integrated - gyro)")
            # ax.plot(time_imu, y_cp, "r-", label="CF: LPF + yaw (integrated gyro)")
            # ax.set_ylabel('Yaw (rad)')
            # ax.set_xlabel('Time (sec)')
            # ax.set_title(shape + ': Yaw Filtered Data')
            # ax.legend(loc = 'upper right')
            ##### PLOT 4: FORWARD VELOCITY FROM ACCELEROMETER BEFORE AND AFTER ADJUSTMENTS #########################

            ###### VELOCITY PLOTS BEFORE CORRECITON #################################################################
            forward_acc = acc[0] 
            forward_vel_imu = integrate.cumulative_trapezoid(forward_acc, time_imu, initial = 0)

            ###### VELOCITY PLOTS AFTER CORRECTION ##################################################################
            forward_acc_corrected = acc[0] + 0.207 # CORRECTION APPLIED AS CONSTANT TO ACCELERATION
            forward_vel_imu_corrected = integrate.cumulative_trapezoid(forward_acc_corrected, time_imu, initial = 0)

            # Calculating velocity estimate from gps
            # Velcity estimate obtained from euclidean distance/ time difference at each interval
            euclidean_dist = []
            
            for i in range(len(northing)-1):
                euclidean_dist.append(math.sqrt((northing[i+1] - northing[i])**2 + (easting[i+1]-easting[i])**2))

            euclidean_dist = np.array(euclidean_dist)

            time_diff = []

            for i in range(len(time_gps)-1):
                time_diff.append(time_gps[i+1]-time_gps[i])

            time_diff = np.array(time_diff)

            forward_vel_gps = euclidean_dist/time_diff
            forward_vel_gps = np.insert(forward_vel_gps, 0, 0)

            ##### PLOT FOR RESULT COMPARISON (COMMENT OUT LATER JUST FOR TESTING) ################################
            fig, ax = plt.subplots(1, figsize=(15,8))
            # ax.plot(time_imu, forward_vel_imu, color = 'blue', label="Velocity (integrated)")
            ax.plot(time_imu, forward_vel_imu_corrected, color = 'green', linestyle = 'dashed', label="Velocity (integrated imu) - corrected")
            ax.plot(time_gps, forward_vel_gps, color = 'red', label="Velocity (integrated gps)")
            ax.set_ylabel('Velocity (m/s)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': GPS Velocity Plot')
            ax.legend(loc = 'upper right')

            ###### IMU VELOCITY PLOTS ###############################################################################
            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_imu, forward_vel_imu, color = 'blue', label="Velocity (integrated)")
            ax.plot(time_imu, forward_vel_imu_corrected, color = 'green', label="Velocity (integrated) - corrected")
            ax.set_ylabel('Velocity (m/s)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': IMU Velocity Plots')
            ax.legend(loc = 'upper right')

            ##### PLOT 5: GPS VELOCITY PLOTS #########################################################################
            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_gps, forward_vel_gps, color = 'red', label="Velocity (integrated gps)")
            ax.set_ylabel('Velocity (m/s)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': GPS Velocity Plots')
            ax.legend(loc = 'upper right')

            displacement_imu = integrate.cumulative_trapezoid(forward_vel_imu_corrected, time_imu, initial=0)
            displacement_gps = integrate.cumulative_trapezoid(forward_vel_gps, time_gps, initial=0)

            # Displacement plots to compare integrated displacements from gps and imu
            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_gps, displacement_gps, color='blue',label="Displacement - gps")
            ax.plot(time_imu, displacement_imu, color='green',label="Displacement - imu")
            ax.set_ylabel('Displacement (m)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': Displacement Plots')
            ax.legend(loc='upper right')

            ##### Comparison of y" observed and y" estimated (yaw*velocity) ##################################
            yaw_rate = ang[2]
            y_acc_estimated = yaw_rate*forward_vel_imu_corrected
            y_acc_observed = acc[1]

            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(time_imu, y_acc_observed, color='green',label="y'' - imu")
            ax.plot(time_imu, y_acc_estimated, color='blue',label="w*x'")
            ax.set_ylabel('Acceleration (m/s^2)')
            ax.set_xlabel('Time (sec)')
            ax.set_title(shape + ': Acceleration in Y Plots')
            ax.legend(loc='upper right')

            velocity_n_imu = np.cos(y_cp*-1)*forward_vel_imu_corrected
            velocity_e_imu = np.sin(y_cp*-1)*forward_vel_imu_corrected

            position_n_imu = integrate.cumulative_trapezoid(velocity_n_imu, time_imu, initial=0)
            position_e_imu = integrate.cumulative_trapezoid(velocity_e_imu, time_imu, initial=0)

            fig, ax = plt.subplots(1, figsize=(15,8))
            ax.plot(position_e_imu, position_n_imu, color='blue',label="Position - imu estimated")
            ax.plot(northing, easting, color='green',label="Position - GPS")
            ax.set_ylabel('Northing (m)')
            ax.set_xlabel('Easting (m)')
            ax.set_title(shape + ': Northing vs. Easting Plots')
            ax.legend(loc='upper right')


            




            plt.show()





        # fig, ax = plt.subplots(3, figsize=(15,8))
        # ax[0].scatter(time, ang[0], color = 'red', label="X-axis")
        # ax[1].scatter(time, ang[1], color = 'green', label="Y-axis")
        # ax[2].scatter(time, ang[2], color = 'blue', label="Z-axis")
        # ax[1].set_ylabel('Rotational Rate (deg/sec)')
        # ax[2].set_xlabel('Time (sec)')
        # ax[0].set_title(shape + ': Rotational Rate vs. Time')

        # min_rot = min([min(rot) for rot in ang])
        # max_rot = max([max(rot) for rot in ang])

        # for axs in ax:
        #     axs.legend(loc = 'upper right')
        #     axs.set_ylim(min_rot-10, max_rot+10)


        # rotation = integrate.cumulative_trapezoid(ang, time, initial = 0)

        # fig, ax = plt.subplots(3, figsize=(15,8))
        # ax[0].scatter(time, rotation[0], color = 'red', label="Roll")
        # ax[1].scatter(time, rotation[1], color = 'green', label="Pitch")
        # ax[2].scatter(time, rotation[2], color = 'blue', label="Yaw")
        # ax[1].set_ylabel('Rotation (deg)')
        # ax[2].set_xlabel('Time (sec)')
        # ax[0].set_title(shape + ': Rotation vs. Time')
        # for axs in ax:
        #     axs.legend(loc = 'upper right')

        # mag_heading = np.arctan(X_meas[0]/X_meas[1])

        # fig, ax = plt.subplots()
        # ax.scatter(time, mag_heading, label="Magnetometer Heading")
        # ax.legend()
        # plt.xlabel('Time (sec)')
        # plt.ylabel('Magnetometer Heading (radians)')
        # plt.title(shape + ': Magnetometer Heading vs. Time')

        # fig, ax = plt.subplots(3, figsize=(15,8))
        # ax[0].scatter(time, acc[0], color = 'red', label="X-axis")
        # ax[1].scatter(time, acc[1], color = 'green', label="Y-axis")
        # ax[2].scatter(time, acc[2], color = 'blue', label="Z-axis")
        # ax[1].set_ylabel('Acceleration (m/$s^2$)')
        # ax[2].set_xlabel('Time (sec)')
        # ax[0].set_title(shape + ': Acceleration vs. Time')

        # min_acc = min([min(x) for x in acc])
        # max_acc = max([max(x) for x in acc])

        # for axs in ax:
        #     axs.legend(loc = 'upper right')
        #     axs.set_ylim(min_acc-1, max_acc+1)


        # velocity = integrate.cumulative_trapezoid(acc, time, initial = 0)
        # displacement = integrate.cumulative_trapezoid(velocity, time, initial = 0)

        # fig, ax = plt.subplots(3, figsize=(15,8))
        # ax[0].scatter(time, velocity[0], color = 'red', label="X-axis")
        # ax[1].scatter(time, velocity[1], color = 'green', label="Y-axis")
        # ax[2].scatter(time, velocity[2], color = 'blue', label="Z-axis")
        # ax[1].set_ylabel('Velocity (m/s)')
        # ax[2].set_xlabel('Time (sec)')
        # ax[0].set_title(shape + ': Velocity vs. Time')

        # min_vel = min([min(x) for x in velocity])
        # max_vel = max([max(x) for x in velocity])       

        # for axs in ax:
        #     axs.legend(loc = 'upper right')
        #     axs.set_ylim(min_vel-1000,max_vel+1000)

        # fig, ax = plt.subplots(3, figsize=(15,8))
        # ax[0].scatter(time, displacement[0], color = 'red', label="X-axis")
        # ax[1].scatter(time, displacement[1], color = 'green', label="Y-axis")
        # ax[2].scatter(time, displacement[2], color = 'blue', label="Z-axis")
        # ax[1].set_ylabel('Displacement (m)')
        # ax[2].set_xlabel('Time (sec)')
        # ax[0].set_title(shape + ': Displacement vs. Time')

        # min_d = min([min(x) for x in displacement])
        # max_d = max([max(x) for x in displacement])  

        # for axs in ax:
        #     axs.legend(loc = 'upper right')
        #     axs.set_ylim(min_d - 0.1e6,max_d + 0.1e6)

