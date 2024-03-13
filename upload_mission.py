from pymavlink import mavutil
from pymavlink import mavwp
import time
from sys import platform

# Create the connection
# From topside computer

# master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600)
master.wait_heartbeat()

wp = mavwp.MAVWPLoader()

def cmd_set_home(home_location, altitude):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

def uploadmission(aFileName):
    home_location = None
    home_altitude = None
    print("hereeeeeeeeeeeeeeeeeeeeeeeeee #2")

    with open(aFileName) as f:
        print("hereeeeereeee #3")
        for i, line in enumerate(f):
            print("hereeeeereeee #3.2")
            if i==0:
                print("hereeeeereeee #3.3")
                if not line.startswith('QGC WPL 110'):
                    print("hereeeeereeee #3.4")
                    raise Exception('File is not supported WP version')
            else: 
                print("heereeeeeeeeeeeeeee #4")  
                linearray=line.split('\t')
                print(str(linearray))
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    print("hereeeeereeee #5")
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                    
    cmd_set_home(home_location,home_altitude)
    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True, timeout = 10)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)

    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

def execute_upload():
    if platform == 'linux':
        print("hereeeee #1 ")
        uploadmission('/Users/labeast021/Desktop/GUI/Aero2024/mav_waypoints.txt')
    else:
        uploadmission('/Users/labeast021/Desktop/GUI/Aero2024/mav_waypoints.txt')
    print('done')