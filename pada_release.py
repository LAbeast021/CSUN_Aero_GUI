import serial
from pymavlink import mavutil
from dronekit import connect
# from flask import Flask, render_template
from pymavlink.dialects.v20 import common

# mav_conn = mavutil.mavlink_connection('/dev/ttyTHS0', baud=57600)  # Adjust the device path and baud rate as needed
mav_conn2 = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600) 
# mav_conn.wait_heartbeat()
mav_conn2.wait_heartbeat()



# #
# mav_conn2.mav.command_long_send(mav_conn2.target_system, mav_conn2.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
# msg = mav_conn2.recv_match(type='COMMAND_ACK' ,blocking=True)
# print(msg) #change the type to any required info or remove type completly for all the information

open = 1280
close = 1701
color = 1505
yellow = 1805

mav_conn2.mav.command_long_send(mav_conn2.target_system, mav_conn2.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, 7, close, 0, 0 , 0, 0, 0)
msg = mav_conn2.recv_match(type='COMMAND_ACK' ,blocking=True)
print(msg)#change the type to any required info or remove type completly for all the information


mav_conn2.mav.command_long_send(mav_conn2.target_system, mav_conn2.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, 8, color, 0, 0 , 0, 0, 0)
msg = mav_conn2.recv_match(type='COMMAND_ACK' ,blocking=True)
print(msg)#change the type to any required info or remove type completly for all the information

mav_conn2.mav.command_long_send(mav_conn2.target_system, mav_conn2.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, 9, 1280 , 0, 0 , 0, 0, 0)
msg = mav_conn2.recv_match(type='COMMAND_ACK' ,blocking=True)
print(msg)#change the type to any required info or remove type completly for all the information


# otherMsg = mav_conn.recv_match(type='SERVO_OUTPUT_RAW' ,blocking=True) #change the type to any required info or remove type completly for all the information
# print("------------------------------------------------------------------------------------------------")
# print(otherMsg)

# while True:
#     msg = mav_conn.recv_msg()
#     otherMsg = mav_conn.recv_match(type='SERVO_OUTPUT_RAW' ,blocking=True) #change the type to any required info or remove type completly for all the information
#     print("------------------------------------------------------------------------------------------------")
#     print(otherMsg)
#     print("HEARTBEAT FROM SYSTEM (SYSTEM %u - COMPONENT %u )" % (mav_conn.target_system , mav_conn.target_component))

#     print("________________________________________________________________________________________________")
#     if msg and msg.get_type() == 'HEARTBEAT':
#         print("HEARTBEAT FROM SYSTEM (SYSTEM %u - COMPONENT %u )" % (mav_conn.target_system , mav_conn.target_component))
#         print("Received HEARTBEAT message: Type = {}, Autopilot = {}, Base Mode = {}".format(
#             msg.type, msg.autopilot, msg.base_mode))
#         break





# # Connect to the vehicle (Orange Cube)
# vehicle = connect('/dev/ttyTHS0', wait_ready=True, baud=07600)

# # Wait for the heartbeat message
# @vehicle.on_message('HEARTBEAT')
# def heartbeat_callback(self, name, message):
#     print("Received HEARTBEAT message: Type = {}, Autopilot = {}, Base Mode = {}".format(
#         message.type, message.autopilot, message.base_mode))

# # Wait for the heartbeat message for a certain amount of time
# import time
# timeout = 30  # Timeout in seconds
# start_time = time.time()
# while time.time() - start_time < timeout:
#     time.sleep(1)

# # Close the connection
# vehicle.close()


# while True:
#     try:
#         msg = mav_conn.recv_msg()
#         if msg is not None and msg.get_type() == 'HEARTBEAT' :
#             # This is a heartbeat message.
#             print("Received a heartbeat message:")
#             print(f"Type: {msg.type}")
#             print(f"Autopilot: {msg.autopilot}")
#             print(f"Base mode: {msg.base_mode}")
#             print(f"Custom mode: {msg.custom_mode}")
#             print(f"System status: {msg.system_status}")
#             print(f"MAVLink version: {msg.mavlink_version}")
#         else :
#             print("somethin went wrong")    
#     except KeyboardInterrupt:
#         break
