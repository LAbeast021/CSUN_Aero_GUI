from pymavlink import mavutil
from time import sleep

mavconn = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=57600)
mavconn.wait_heartbeat()
print("Connected to vehicle")


# mavconn.mav.command_long_send(
#     mavconn.target_system, mavconn.target_component,
#     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
#     109, 1000000 / 10, 0, 0, 0, 0, 0, 0
# )
while True:
    msg = mavconn.recv_match(type=['STATUSTEXT'], blocking=True , timeout=1) 
    if msg is not None:
        print(msg)

    else:
        print("No message received")
        