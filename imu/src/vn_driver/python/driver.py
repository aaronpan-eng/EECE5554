#!/usr/bin/env python

from pickletools import uint8
import rospy
import utm
import time
import serial
import sys
import numpy as np
import math

from vn_driver.msg import Vectornav

###FUNCTIONS###
def convert_to_quaternion_c(yaw, pitch, roll):
    cy = math.cos(yaw)
    cp = math.cos(pitch)
    cr = math.cos(roll)
    sy = math.sin(yaw)
    sp = math.sin(pitch)
    sr = math.sin(roll)

    C = np.array([[cp*cy, cp*sy, -sp],
                   [(sr*sp*cy)-(cr*sy), (sr*sp*sy)+(cr*cy), sr*cp],
                   [(cr*sp*cy)+(sr*sy), (cr*sp*sy)-(sr*cy), cr*cp]])
    
    q1 = np.sqrt((1+C[0,0]-C[1,1]-C[2,2])/4)
    q2 = np.sqrt((1-C[0,0]+C[1,1]-C[2,2])/4)
    q3 = np.sqrt((1-C[0,0]-C[1,1]+C[2,2])/4)
    q4 = np.sqrt((1+C[0,0]+C[1,1]+C[2,2])/4)
    
    q = [q1,q2,q3,q4]

    max_num = max(q)

    max_index = q.index(max_num)

    if max_index == 0:
        q[1] = (C[0,1]+C[1,0])/(4*q[max_index])
        q[2] = (C[2,0]+C[0,2])/(4*q[max_index])
        q[3] = (C[1,2]-C[2,1])/(4*q[max_index])
    elif max_index == 1:
        q[0] = (C[0,1]+C[1,0])/(4*q[max_index])
        q[2] = (C[1,2]+C[2,1])/(4*q[max_index])
        q[3] = (C[2,0]-C[0,2])/(4*q[max_index])
    elif max_index == 2:
        q[0] = (C[2,0]+C[0,2])/(4*q[max_index])
        q[1] = (C[1,2]+C[2,1])/(4*q[max_index])
        q[3] = (C[0,1]-C[1,0])/(4*q[max_index])
    elif max_index == 3:
        q[0] = (C[1,2]-C[2,1])/(4*q[max_index])
        q[1] = (C[2,0]-C[0,2])/(4*q[max_index])
        q[2] = (C[0,1]-C[1,0])/(4*q[max_index])

    return [q1, q2, q3, q4]

def convert_to_quaternion_s(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

###MAIN CODE###
if __name__ == '__main__':

    #initializing port to argument from terminal
    port = sys.argv[1]
    customimu = Vectornav()
    # customimu.header.seq = 0

    #intializing node
    rospy.init_node('vn_driver')
    
    #initializing port params
    serial_port = rospy.get_param(port,'/dev/pts/5')
    serial_baud = rospy.get_param('~baudrate',115200)
    port = serial.Serial(port, serial_baud, timeout=3.)
    # port.write(b"$VNWRG,07,40*xx")

    rospy.logdebug("Using IMU on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor")

    #initialize publisher
    customimu_pub = rospy.Publisher('/imu', Vectornav, queue_size = 5)

    try:
        while not rospy.is_shutdown():
            #read line from port
            vnRead = port.readline()
            vnRead = str(vnRead)

            #split by commas
            vnSplit = vnRead.split(",") 
            # print(vnSplit)

            #error checking
            if vnSplit[2] != '' and (vnSplit[0] == "b'$VNYMR" or vnSplit[0] == "b'\\r$VNYMR"):
            #     rospy.logwarn("VN data not good or had no connection")
            # else:
                yaw = np.radians(float(vnSplit[1]))
                pitch = np.radians(float(vnSplit[2]))
                roll = np.radians(float(vnSplit[3]))
                mag_x = float(vnSplit[4])
                mag_y = float(vnSplit[5])
                mag_z = float(vnSplit[6])
                acc_x = float(vnSplit[7])
                acc_y = float(vnSplit[8])
                acc_z = float(vnSplit[9])
                ang_x = float(vnSplit[10])
                ang_y = float(vnSplit[11])

                lastSplit = vnSplit[12].split("*")

                ang_z = float(lastSplit[0])
                # checksum = str(lastSplit[1])
                
                qx, qy, qz, qw = convert_to_quaternion_c(yaw, pitch, roll)

                currentTime = rospy.Time.now()

                customimu.header.frame_id = 'imu1_frame'
                customimu.header.stamp.secs = currentTime.secs
                customimu.header.stamp.nsecs = currentTime.nsecs
                customimu.imu.orientation.x = qx
                customimu.imu.orientation.y = qy
                customimu.imu.orientation.z = qz
                customimu.imu.orientation.w = qw
                customimu.imu.angular_velocity.x = ang_x
                customimu.imu.angular_velocity.y = ang_y
                customimu.imu.angular_velocity.z = ang_z
                customimu.imu.linear_acceleration.x = acc_x
                customimu.imu.linear_acceleration.y = acc_y
                customimu.imu.linear_acceleration.z = acc_z
                customimu.mag_field.magnetic_field.x = mag_x
                customimu.mag_field.magnetic_field.y = mag_y
                customimu.mag_field.magnetic_field.z = mag_z
                customimu.rawIMU = vnRead

                # customimu.header.seq+=1

                #publishing
                customimu_pub.publish(customimu)
                print(customimu)
            
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
