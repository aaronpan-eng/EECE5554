#!/usr/bin/env python

from pickletools import uint8
import rospy
import utm
import time
import serial
import sys

from gps_driver.msg import Customgps

###FUNCTIONS###

# def isGPGGAinString(inputString):
#     if 1 == 1: #replace 1 == 1 with condition to be checked for inputString
#         print('')
#     else:
#         print('')

def degMinstoDegDec(LatOrLong):
    deg = int(LatOrLong/100) #Replace 0 with a line of code that gets just the degrees from LatOrLong
    mins = LatOrLong - (deg*100) #Replace 0 with a line of code that gets just the minutes from LatOrLong
    degDec = mins/60.0 #Replace 0 with a line of code that converts minutes to decimal degrees
    print(deg+degDec)
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir == "W" or LatOrLongDir == "S": #Replace the blank string with a value
        LatOrLong *= -1 #some code here that applies negative convention
        print(LatOrLong)
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0] #Again, replace these with values from UTMVals
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    print(UTMVals)
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

# def ReadFromSerial(serialPortAddr):
#     serialPort = serial.Serial(serialPortAddr) #This line opens the port, do not modify
#     gpggaRead = serialPort.readline() #Replace this line with a 1-line code to read from the serial port
#     print(gpggaRead)
#     serialPort.close() #Do not modify
#     return gpggaRead



###MAIN CODE###
if __name__ == '__main__':

    # stringReadfromPort = ''
    # isGPGGAinString(stringReadfromPort)

    # gpggaRead = '$GPGGA,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61'
    port = sys.argv[1]
    customgps = Customgps()
    
    # port = '/dev/pts/3'
    rospy.init_node('gps_driver')
    
    
    serial_port = rospy.get_param(port,'/dev/pts/5')
    serial_baud = rospy.get_param('~baudrate',4800)
    port = serial.Serial(port, serial_baud, timeout=3.)

    rospy.logdebug("Using GPS on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor")

    customgps_pub = rospy.Publisher('/gps', Customgps, queue_size = 5)

    try:
        while not rospy.is_shutdown():
            # gpggaRead = ReadFromSerial(serialPortAddr) #Do not modify
            
            gpggaRead = port.readline()
            
            # gpggaRead = ReadFromSerial(port)
            gpggaRead = str(gpggaRead)
            # print(gpggaRead)
            gpggaSplit = gpggaRead.split(",") #Put code here that will split gpggaRead into its components. This should only take one line.
            # print(gpggaSplit)
            # print(gpggaSplit[0])
            # print(gpggaSplit[1])

            if gpggaSplit[2] == '' and (gpggaSplit[0] == "b'$GPGGA" or gpggaSplit[0] == "b'\\r$GPGGA"):
                rospy.logwarn("GPS doesn't have good connection")
            if gpggaSplit[0] == "b'$GPGGA" or gpggaSplit[0] == "b'\\r$GPGGA":
                UTC = float(gpggaSplit[1]) #float
                Latitude = float(gpggaSplit[2]) #float
                LatitudeDir = str(gpggaSplit[3]) #string
                Longitude = float(gpggaSplit[4]) #float
                LongitudeDir = str(gpggaSplit[5]) #string
                HDOP = float(gpggaSplit[8]) #float
                Altitude = float(gpggaSplit[9]) #float
                print(Latitude)

                degDec_Latitude = degMinstoDegDec(Latitude)
                degDec_Longitude = degMinstoDegDec(Longitude)

                LatitudeSigned = LatLongSignConvetion(degDec_Latitude, LatitudeDir)
                LongitudeSigned = LatLongSignConvetion(degDec_Longitude, LongitudeDir)

                UTMEasting, UTMNorthing, UTMZone, UTMLetter = convertToUTM(LatitudeSigned, LongitudeSigned)

                CurrentTimeSec, CurrentTimeNsec = UTCtoUTCEpoch(UTC)

                customgps.header.frame_id = 'GPS1_Frame'
                customgps.header.stamp.secs = CurrentTimeSec
                customgps.header.stamp.nsecs = CurrentTimeNsec
                customgps.latitude = LatitudeSigned
                customgps.longitude = LongitudeSigned
                customgps.altitude = Altitude
                customgps.utm_easting = UTMEasting
                customgps.utm_northing = UTMNorthing
                customgps.zone = UTMZone
                customgps.letter = UTMLetter
                customgps.hdop = HDOP
                customgps.gpgga_read = gpggaRead    

                customgps_pub.publish(customgps)
                print(customgps)
                # rospy.sleep()
    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
    # serialPortAddr = '/dev/ttyUSB0' #You will need to change this to the emulator or GPS puck port
    
    # lat_pub = rospy.Publisher("customgps"+'/latitude', Float64, queue_size = 10)
    # long_pub = rospy.Publisher("customgps"+'/longitude', Float64, queue_size = 10)
    # utmeasting_pub = rospy.Publisher("customgps"+'/utm_easting', Float64, queue_size = 10)
    # utmnorthing_pub = rospy.Publisher("customgps"+'/utm_northing', Float64, queue_size = 10)
    # utmzone_pub = rospy.Publisher("customgps"+'/utm_zone', uint8, queue_size = 10)
    # utmletter_pub = rospy.Publisher("customgps"+'/utm_letter', string, queue_size = 10)
    
    