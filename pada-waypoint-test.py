
import threading
import time
from pymavlink import mavutil

PADA_conn = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600) # for macos
PADA_conn.wait_heartbeat()
print("connected")


def sendMissionToPADA(lat, lon):
    from waypoint_2_mav import waypoint_gen_format
    from upload_mission import execute_upload

    # Define wrapper functions for the operations you want to perform in threads
    def thread_waypoint_gen_format(lat, lon):
        waypoint_gen_format(lat, lon)

    def thread_execute_upload():
        execute_upload()
        
    # Create the first thread for waypoint_gen_format
    thread1 = threading.Thread(target=thread_waypoint_gen_format, args=(lat, lon))

    # Start the first thread
    thread1.start()

    # Wait for the first thread to complete
    thread1.join()

    # After the first thread completes, create and start the second thread for execute_upload
    thread2 = threading.Thread(target=thread_execute_upload)
    thread2.start()

    # Wait for the second thread to complete, if needed (for example, if there are further actions that depend on this completion)
    thread2.join()



sendMissionToPADA(34.24204963058731 , -118.52890962461137)    