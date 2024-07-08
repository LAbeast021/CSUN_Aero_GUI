# from precision_function import precision
from pymavlink import mavutil
from pymavlink import mavwp
from time import sleep

from waypoint_generator import waypointGenerator



global master
master = None

global logWindow
logWindow = None

def upload_mission_from_array(waypoints , fieldAlt):

    waypointArray = waypoints
    success = False
    max_retries_per_waypoint = 4
    # master = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600)
    # master.wait_heartbeat()
    # print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    while success == False:
        try:
            # master = mavutil.mavlink_connection('/dev/tty.usbserial-AK06O4AL', baud=115200)
            # heartbeat = master.wait_heartbeat(timeout=5)

            master.waypoint_clear_all_send()  # Clear existing mission
            sleep(2) 
            print("Cleared the existing waypoints .")

            master.waypoint_count_send(len(waypointArray))
            sleep(1)
            
            retry_count = 0
            all_waypoints_sent = False
            waypoint_sent_successfully = False

            # retry_count < max_retries_per_waypoint
            while not waypoint_sent_successfully:
                # # Convert waypoint data to the expected types

                # Wait for the vehicle to request each waypoint
                msg = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)

                if msg is not None:
                    print(f"Vehicle requested waypoint {msg.seq}")
                    logWindow.insert(1.0, f"Vehicle requested waypoint {msg.seq} !\n")

                    requestedMission = waypointArray[msg.seq]

                    seq, current, frame, command = int(requestedMission[0]), int(requestedMission[1]), int(requestedMission[2]), int(requestedMission[3])
                    param1, param2, param3, param4 = float(requestedMission[4]), float(requestedMission[5]), float(requestedMission[6]), float(requestedMission[7])
                    x, y, z = float(requestedMission[8]), float(requestedMission[9]), float(requestedMission[10])
                    autocontinue = int(requestedMission[11])
                    # Create and send the mission item
                    master.mav.mission_item_send(master.target_system, master.target_component,
                                                seq, frame, command,
                                                current, autocontinue, param1, param2, param3, param4,
                                                x, y, z)
                    print(f"Sending waypoint {seq}: Lat {x}, Lon {y}, Alt {z}")
                    logWindow.insert(1.0, f"Sending waypoint {seq}: Lat {x}, Lon {y}, Alt {z} \n", "color4")
                    fieldAlt = fieldAlt - 10
                    if seq == len(waypointArray) - 1:
                        ack = master.recv_match(type='MISSION_ACK', blocking=True , timeout=4)
                        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            if msg.seq >= 3:
                                all_waypoints_sent = True
                            print("Mission successfully uploaded.")
                            logWindow.insert(1.0, f" Mission successfully uploaded To PADA !\n")
                            # success = True
                            # trying = max_try
                            print("Last waypoint sent.")
                            # waypoint_sent_successfully = True
                        else:
                            print(f"Mission upload failed with error: {ack.type}" if ack else "No MISSION_ACK received.")
                            if ack:
                                logWindow.insert(1.0, f"Mission upload failed with error: {ack.type}\n")
                            else:
                                logWindow.insert(1.0, f" No MISSION_ACK received. !\n")

                elif msg is None and all_waypoints_sent:
                    # trying = 0
                    # max_try = 10
                    # Wait for mission acknowledgment only if all waypoints were sent successfully
                    # while trying < max_try and not success:
                    #         trying += 1
                    success = True
                    waypoint_sent_successfully = True

                elif msg is None:
                    print("No MISSION_REQUEST received.")
                    retry_count += 1
                    if retry_count >= max_retries_per_waypoint:
                        print(f"Failed to receive MISSION_REQUEST after {max_retries_per_waypoint} retries.")
                        break

            if not waypoint_sent_successfully:
                print(f"Failed to send waypoint after {max_retries_per_waypoint} retries. Aborting mission upload.")
                all_waypoints_sent = False
                break  # Exit the loop if a waypoint fails to send

        except Exception as error:
            print (f"failed with error {error} . trying again ")
    return master


def precision(variable, precision_level): 
    chars = "."
    has = all(char in str(variable) for char in chars)
    if not has:
        variable = str(variable) + "."
    x = str(variable).split(".")
    output = (x[0] + "." + (x[1]).ljust(precision_level, "0"))
    return output

def to_float(precision_output):
    return float(precision_output)


# Initialize the waypoint generator with the provided coordinates
def create_waypoints(lat, lon, direction_code , PADA_Conn , log , fieldAlt):
    global master
    global logWindow

    print("we are in waypoints wohooooo")
    coordinates = waypointGenerator(lat, lon, 0 ,direction_code)

    # Generate waypoints in arrays
    waypoints = [
        ["0", "1", "3", "16", "0", "0", "0", "0", to_float(precision(coordinates.waypoint()[0], 7)), to_float(precision(coordinates.waypoint()[1], 7)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["1", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.waypoint()[0], 8)), to_float(precision(coordinates.waypoint()[1], 8)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["2", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.find_midpoint()[0], 8)), to_float(precision(coordinates.find_midpoint()[1], 8)), to_float(precision(coordinates.find_midpoint()[2], 6)), "1"],
        ["3", "0", "3", "21", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.originalCoords()[0], 8)), to_float(precision(coordinates.originalCoords()[1], 8)), to_float(precision(coordinates.originalCoords()[2], 6)), "1"]
    ]

    master = PADA_Conn
    logWindow = log

    upload_mission_from_array(waypoints , fieldAlt)

