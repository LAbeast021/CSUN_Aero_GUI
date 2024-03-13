import time
from pymavlink import mavutil

mav_conn = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=57600) # for macOS
mav_conn.wait_heartbeat()
print("connected")

# PADA_conn = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600) # for macos
# PADA_conn.wait_heartbeat()
# print("connected")


def setServoandRelease() : 
    # mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7 , 900 , 0 , 0, 0, 0, 0)
    # msg = mav_conn.recv_match(type='COMMAND_ACK' ,blocking=True)
    mav_conn.mav.command_long_send(mav_conn.target_system, mav_conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7 , 2100 , 0 , 0, 0, 0, 0)
    print("setting servo")
    time.sleep(1.5)
    mode = 'AUTO'  # Define the mode you want to set; for ArduPilot, AUTO mode for autonomous operation

    # # Find the mode ID for AUTO mode
    # mode_id = PADA_conn.mode_mapping()[mode]

    # # Set the mode
    # PADA_conn.mav.set_mode_send(
    #     PADA_conn.target_system,
    #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    #     mode_id)
    # =========================================================================
     mav_conn.waypoint_request_list_send()
    servo_msg_for_mission = mav_conn.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)

    if servo_msg_for_mission is None and triesForMission < 3:
        log.insert(1.0, f"Failed to receive mission count {triesForMission}\n", "color3")
        for i in range(len(event_listener) - 1, -1, -1):  # Start from the last index, go to 0
            if 1440 in event_listener[i]:
                del event_listener[i]  # Use del to remove the item at index i
                triesForMission += 1
                break
    elif servo_msg_for_mission:
        log.insert(1.0, f"Mission count received: {servo_msg_for_mission.count}\n", "color1")
        missionRecieved = True
        lat = 0.0
        lon = 0.0
        for i in range(servo_msg_for_mission.count):
            print("hereee 4 ")
            mav_conn.waypoint_request_send(i)
            waypoint = mav_conn.recv_match(type='MISSION_ITEM', blocking=True)
            print(waypoint)
            # print(f"Waypoint {i + 1}: Latitude={waypoint.x}, Longitude={waypoint.y}, Altitude={waypoint.z}")
            if i == servo_msg_for_mission.count - 1:
                print("hereee 5 ")
                current_color = colorArray[colorPWM.index(servo_msg.servo8_raw)]
                log_text = f"Mission Received coordinates for target {current_color} -> lat: {waypoint.x}, lon: {waypoint.y}\n"
                log.insert(1.0, log_text)
                lat = waypoint.x
                lon = waypoint.y
                print(lat)
                print(lon)

    

setServoandRelease()
print("done")
# time.sleep(5)


# def putToAuto ():

#     mode = 'MANUAL'  # Define the mode you want to set; for ArduPilot, AUTO mode for autonomous operation

#     # Find the mode ID for AUTO mode
#     mode_id = PADA_conn.mode_mapping()[mode]

#     # Set the mode
#     PADA_conn.mav.set_mode_send(
#         PADA_conn.target_system,
#         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#         mode_id)
