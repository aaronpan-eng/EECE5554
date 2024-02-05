import rosbag
import bagpy
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

# from gps_driver.msg import Customgps

#function to take whats needed from the data
def process_data(file):
    #data that will be returned
    easting = []
    northing = []
    altitude = []
    time = []
    cent_easting = 0.0
    cent_northing = 0.0

    with rosbag.Bag(file) as bag:
        northing_tot = 0.0
        easting_tot = 0.0
        pts = 0
        northing_first = 0.0
        easting_first = 0.0

        #stepping thru each line in the bag file and extracting data needed
        for topic, msg, t in bag.read_messages(topics = "/gps"):
            east_msg = msg.utm_easting
            north_msg = msg.utm_northing
            alt_msg = msg.altitude
            time_msg = msg.header.stamp.secs

            if pts == 0:
                northing_first = north_msg
                easting_first = east_msg
            
            #subtracting first value of northing and easting from dataset to normalize
            north_msg -= northing_first
            east_msg -= easting_first

            northing_tot += north_msg
            easting_tot += east_msg
            pts += 1

            northing.append(north_msg)
            easting.append(east_msg)
            altitude.append(alt_msg)
            time.append(time_msg)
        
        #centroid calc
        cent_northing = northing_tot/pts
        cent_easting = easting_tot/pts

    return[easting, northing, altitude, time, cent_northing, cent_easting]

#processing data for occluded, unoccluded, and walking
occ_easting, occ_northing, occ_altitude, occ_time, occ_cent_northing, occ_cent_easting = process_data('occluded_test.bag')
unocc_easting, unocc_northing, unocc_altitude, unocc_time, unocc_cent_northing, unocc_cent_easting = process_data('unoccluded_test.bag')
walk_easting, walk_northing, walk_altitude, walk_time, walk_cent_northing, walk_cent_easting = process_data('walking_test.bag')

#occluded and unoccluded northing vs easting
plt.figure(1, figsize=(10, 8))
plt.scatter(occ_easting, occ_northing, color='red', label='Occluded data')
plt.scatter(unocc_easting, unocc_northing, color='blue', label='Unoccluded data')
plt.scatter(occ_cent_easting, occ_cent_northing, marker='x', color='red', label='Occluded centroid')
plt.scatter(unocc_cent_easting, unocc_cent_northing, marker='x', color='blue', label='Unoccluded centroid')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.title('Stationary Northing vs. Easting')


#occluded and unoccluded alt vs time
#calculations to normalize the time so that the 2 lines can be overlayed
occ_time = np.array(occ_time)
occ_time = occ_time - occ_time[0]
unocc_time = np.array(unocc_time)
unocc_time = unocc_time - unocc_time[0]
occ_altitude = np.array(occ_altitude)
unocc_altitude = np.array(unocc_altitude)

plt.figure(2, figsize=(10, 8))
plt.scatter(occ_time, occ_altitude, color='red', label='Occluded altitude')
plt.scatter(unocc_time, unocc_altitude, color='blue', label='Unoccluded altitude')
plt.xlabel('Timestamp (sec)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.title('Stationary Altitude vs. Time')

#histograms for the distance to centroid (occluded and unoccluded)
#histogram for occluded
distance_occ = np.sqrt((np.array(occ_easting))**2 + (np.array(occ_northing))**2)

plt.figure(3, figsize=(10, 8))
plt.hist(distance_occ, bins = 15, color='red', label='Occluded')
plt.xlim(0,17.5)
plt.xlabel('Euclidean dist to centroid (m)')
plt.ylabel('Frequency')
plt.legend()
plt.title('Stationary Occluded Histogram')

#histogram for unoccluded
distance_unocc = np.sqrt((np.array(unocc_easting))**2 + (np.array(unocc_northing))**2)

plt.figure(4, figsize=(10, 8))
plt.hist(distance_unocc, bins = 15, color='blue', label='Unoccluded')
plt.xlim(0,17.5)
plt.xlabel('Euclidean dist to centroid (m)')
plt.ylabel('Frequency')
plt.legend()
plt.title('Stationary Unoccluded Histogram')


#walking northing vs easting data with line of best fit
walk_easting = np.array(walk_easting)
walk_northing = np.array(walk_northing)

a,b = np.polyfit(walk_easting, walk_northing, 1)

plt.figure(5, figsize=(10, 8))
plt.scatter(walk_easting, walk_northing, color='blue', label='Walking data')
plt.scatter(walk_cent_easting, walk_cent_northing, marker='x', color='red', label='Walking centroid')
plt.plot(walk_easting, a*walk_easting+b, linestyle = '--', color='orange', label = "Line of best fit")
# plt.text(1,17, 'y = ' + '{:2.f}'.format(b) + ' + {:.2f}'.format(a) + 'x', size = 14)
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.title('Walking Northing vs. Easting')

#walking alt vs time
#normalizing calc below
walk_time = np.array(walk_time)
walk_time = walk_time - walk_time[0]
walk_altitude = np.array(walk_altitude)

plt.figure(6, figsize=(10, 8))
plt.scatter(walk_time, walk_altitude, color='blue', label='Walking altitude')
plt.xlabel('Timestamp (sec)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.title('Walking Altitude vs. Time')
plt.show()