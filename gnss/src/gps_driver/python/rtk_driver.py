#!/usr/bin/env python

from pickletools import uint8
import rospy
import utm
import time
import serial
import sys

from gps_driver.msg import Customrtk

###FUNCTIONS###
def degMinstoDegDec(LatOrLong):
    deg = int(LatOrLong/100) #Replace 0 with a line of code that gets just the degrees from LatOrLong
    mins = LatOrLong - (deg*100) #Replace 0 with a line of code that gets just the minutes from LatOrLong
    degDec = mins/60.0 #Replace 0 with a line of code that converts minutes to decimal degrees
    # print(deg+degDec)
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S": #Replace the blank string with a value
        LatOrLong *= -1 #some code here that applies negative convention
        # print(LatOrLong)
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0] #Again, replace these with values from UTMVals
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    # print(UTMVals)
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(UTC):
    UTCinSecs = (int(UTC/10000)*3600)+(int((UTC%10000)/100)*60)+(UTC%100) #Replace with a line that converts the UTC float in hhmmss.ss to seconds as a float
    TimeSinceEpoch = time.mktime(time.localtime()) #Replace with a 1-line method to get time since epoch
    TimeSinceEpochBOD = TimeSinceEpoch - (TimeSinceEpoch % 86400) #Use the time since epoch to get the time since epoch *at the beginning of the day*
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime) #Replace with a 1-line calculation to get total seconds as an integer
    CurrentTimeNsec = int((CurrentTime-CurrentTimeSec)*1e9) #Replace with a 1-line calculation to get remaining nanoseconds as an integer (between CurrentTime and CurrentTimeSec )
    print(CurrentTime)
    return [CurrentTimeSec, CurrentTimeNsec]


###MAIN CODE###
if __name__ == '__main__':

    #initializing port to argument from terminal
    port = sys.argv[1]
    customrtk = Customrtk()
    
    #intializing node
    rospy.init_node('gps_driver')
    
    #initializing port params
    serial_port = rospy.get_param(port,'/dev/pts/5')
    serial_baud = rospy.get_param('~baudrate',4800)
    port = serial.Serial(port, serial_baud, timeout=3.)

    rospy.logdebug("Using GPS on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor")

    #initialize publisher
    customrtk_pub = rospy.Publisher('/gps', Customrtk, queue_size = 5)

    try:
        while not rospy.is_shutdown():
            #read line from port
            gnggaRead = port.readline()
            gnggaRead = str(gnggaRead)

            #split by commas
            gnggaSplit = gnggaRead.split(",") 

            #checking if gngga string is empty
            if gnggaSplit[2] == '' and (gnggaSplit[0] == "b'$GNGGA" or gnggaSplit[0] == "b'\\r$GNGGA"):
                rospy.logwarn("GPS doesn't have good connection")
            #if gngga string not empty then proceed as normal below
            elif gnggaSplit[0] == "b'$GNGGA" or gnggaSplit[0] == "b'\\r$GNGGA":
                UTC = float(gnggaSplit[1]) #float
                Latitude = float(gnggaSplit[2]) #float
                LatitudeDir = str(gnggaSplit[3]) #string
                Longitude = float(gnggaSplit[4]) #float
                LongitudeDir = str(gnggaSplit[5]) #string
                quality = int(gnggaSplit[6]) #int
                HDOP = float(gnggaSplit[8]) #float
                Altitude = float(gnggaSplit[9]) #float
                # print(Latitude)

                #calculations
                degDec_Latitude = degMinstoDegDec(Latitude)
                degDec_Longitude = degMinstoDegDec(Longitude)

                LatitudeSigned = LatLongSignConvetion(degDec_Latitude, LatitudeDir)
                LongitudeSigned = LatLongSignConvetion(degDec_Longitude, LongitudeDir)

                UTMEasting, UTMNorthing, UTMZone, UTMLetter = convertToUTM(LatitudeSigned, LongitudeSigned)

                CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)

                #setting messages to calculated values
                customrtk.header.frame_id = 'GPS1_Frame'
                customrtk.header.stamp.secs = CurrentTimeSec
                customrtk.header.stamp.nsecs = CurrentTimeNsec
                customrtk.latitude = LatitudeSigned
                customrtk.longitude = LongitudeSigned
                customrtk.altitude = Altitude
                customrtk.utm_easting = UTMEasting
                customrtk.utm_northing = UTMNorthing
                customrtk.zone = UTMZone
                customrtk.letter = UTMLetter
                customrtk.hdop = HDOP
                customrtk.fix_quality = quality
                customrtk.gngga_read = gnggaRead

                #publishing
                customrtk_pub.publish(customrtk)
                print(customrtk)
                
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
    