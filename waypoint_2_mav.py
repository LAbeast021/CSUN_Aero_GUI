# Application Name: waypoint_2_mav
# Programmer: Evan Thomas
# Last Updated Date:October 12th, 2022
# Origin Date: October 8th, 2022
# Purpose: To convert the generated coordinates into a waypoint mission file
# Plan of attack: import class to this file. Run the class and access values
#                 at each index and put them in the appropriate spot for 
#                 waypoint file. Impliment a way to have the index increase
#                 along with the other semi static values in the generated
#                 portion.
#Resources: https://mavlink.io/en/file_formats/
#           https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME
#           https://ardupilot.org/mavproxy/docs/uav_configuration/waypoints.html


########################################
# Change Log 
# Make this code get a shared variable from the yolo8_threading.py file "TARGET_GPS" is the vairable to be shared

########################################
'''
QGC WPL <VERSION>
<INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
'''
from precision_function import precision
from waypoint_generator import waypointGenerator
from sys import platform



def waypoint_gen_format(lat , lon):
    print (platform, "waypoint_2_mav.py")
    coordinates = waypointGenerator( lat, lon, 0 )

    if platform == 'linux':
        file = open("/Users/labeast021/Desktop/GUI/Aero2024/mav_waypoints.txt", "w")  
    else:
        file = open("/Users/labeast021/Desktop/GUI/Aero2024/mav_waypoints.txt", "w")
        # file = open(r"C:\Users\csuna\Desktop\Aero Code\Aero2024\mav_waypoints.txt", "w")
    file.write("QGC WPL 110\n")
    file.write("0\t1\t3\t16\t0\t0\t0\t0\t"+str(precision(coordinates.waypoint()[0], 7))+"\t"+str(precision(coordinates.waypoint()[1], 7))+"\t"+str(precision(coordinates.waypoint()[2], 6))+"\t1\n")#make home coords match first waypoint (change precision)

    #index 0-3 are int then index 4 thru 9 are 8 decimal places, index 10 is 6 decimal places
    file.write("1\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t"+str(precision(coordinates.waypoint()[0], 8))+"\t"+str(precision(coordinates.waypoint()[1], 8))+"\t"+str(precision(coordinates.waypoint()[2], 6))+"\t1\n")
    file.write("2\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t"+str(precision(coordinates.find_midpoint()[0], 8))+"\t"+str(precision(coordinates.find_midpoint()[1], 8))+"\t"+str(precision(coordinates.find_midpoint()[2], 6))+"\t1\n")
    file.write("3\t0\t3\t21\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t"+str(precision(coordinates.originalCoords()[0], 8))+"\t"+str(precision(coordinates.originalCoords()[1], 8))+"\t"+str(precision(coordinates.originalCoords()[2], 6))+"\t1")
    print("waypoint file generated")

