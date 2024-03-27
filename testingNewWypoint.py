
import threading
import time
from pymavlink import mavutil

PADA_conn = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600) # for macos
PADA_conn.wait_heartbeat()
print("connected")


def sendMissionToPADA(lat, lon):
    from new_WayPoint_Generator import create_waypoints

    # Define wrapper functions for the operations you want to perform in threads
    def thread_waypoint_gen_format(lat, lon):
        create_waypoints(lat , lon , 3)

    # Create the first thread for waypoint_gen_format
    thread1 = threading.Thread(target=thread_waypoint_gen_format, args=(lat, lon))

    # Start the first thread
    thread1.start()

    # Wait for the first thread to complete
    thread1.join()



sendMissionToPADA(34.1749893 , -118.4814495)    