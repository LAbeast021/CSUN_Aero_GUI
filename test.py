import time
from pymavlink import mavutil

# mav_conn = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=57600) # for macOS
mav_conn = mavutil.mavlink_connection('/dev/tty.usbserial-B000IDOQ', baud=115200) # for macOS
mav_conn.wait_heartbeat()
print("connected")

# PADA_conn = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600) # for macos
# PADA_conn.wait_heartbeat()
# print("connected")


def setServoandRelease() : 
    # mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7 , 900 , 0 , 0, 0, 0, 0)
    # msg = mav_conn.recv_match(type='COMMAND_ACK' ,blocking=True)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1 , 2100 , 0 , 0, 0, 0, 0)
    print("setting servo")
    time.sleep(1.5)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1 , 500 , 0 , 0, 0, 0, 0)
    time.sleep(1.5)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1 , 900 , 0 , 0, 0, 0, 0)
    time.sleep(1.5)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1 , 500 , 0 , 0, 0, 0, 0)
    time.sleep(1.5)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 2 , 500 , 0 , 0, 0, 0, 0)
    time.sleep(1.5)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 2 , 1000 , 0 , 0, 0, 0, 0)

setServoandRelease()