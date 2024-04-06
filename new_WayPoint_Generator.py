# from precision_function import precision
from pymavlink import mavutil
from pymavlink import mavwp
from time import sleep

from waypoint_generator import waypointGenerator



def upload_mission_from_array(waypoints):

    waypointArray = waypoints
    success = False
    max_retries_per_waypoint = 3

    while success == False:

        master = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600)
        master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
        master.waypoint_clear_all_send()  # Clear existing mission
        sleep(2) 

        master.waypoint_count_send(len(waypointArray))
        all_waypoints_sent = True
        for waypoint in waypointArray:
            retry_count = 0
            waypoint_sent_successfully = False

            while retry_count < max_retries_per_waypoint and not waypoint_sent_successfully:
                # Convert waypoint data to the expected types
                seq, current, frame, command = int(waypoint[0]), int(waypoint[1]), int(waypoint[2]), int(waypoint[3])
                param1, param2, param3, param4 = float(waypoint[4]), float(waypoint[5]), float(waypoint[6]), float(waypoint[7])
                x, y, z = float(waypoint[8]), float(waypoint[9]), float(waypoint[10])
                autocontinue = int(waypoint[11])

                # Wait for the vehicle to request each waypoint
                sleep(1)
                msg = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
                if msg is not None:
                    print(f"Vehicle requested waypoint {msg.seq}")

                    # Create and send the mission item
                    master.mav.mission_item_send(master.target_system, master.target_component,
                                                seq, frame, command,
                                                current, autocontinue, param1, param2, param3, param4,
                                                x, y, z)
                    print(f"Sending waypoint {seq}: Lat {x}, Lon {y}, Alt {z}")
                    waypoint_sent_successfully = True
                else:
                    print(f"Timeout waiting for vehicle to request waypoint. Retry {retry_count + 1} of {max_retries_per_waypoint}")
                    retry_count += 1  # Increment retry count

            if not waypoint_sent_successfully:
                print(f"Failed to send waypoint after {max_retries_per_waypoint} retries. Aborting mission upload.")
                all_waypoints_sent = False
                break  # Exit the loop if a waypoint fails to send

        if all_waypoints_sent:
            # Wait for mission acknowledgment only if all waypoints were sent successfully
            ack = master.recv_match(type='MISSION_ACK', blocking=True , timeout=5)
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Mission successfully uploaded.")
                success = True
            else:
                print(f"Mission upload failed with error: {ack.type}" if ack else "No MISSION_ACK received.")
    master.close()



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
def create_waypoints(lat, lon , direction_code):
    print("we are in waypoints wohooooo")
    coordinates = waypointGenerator(lat, lon, 0 ,direction_code)

    # Generate waypoints in arrays
    waypoints = [
        ["0", "1", "3", "16", "0", "0", "0", "0", to_float(precision(coordinates.waypoint()[0], 7)), to_float(precision(coordinates.waypoint()[1], 7)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["1", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.waypoint()[0], 8)), to_float(precision(coordinates.waypoint()[1], 8)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["2", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.find_midpoint()[0], 8)), to_float(precision(coordinates.find_midpoint()[1], 8)), to_float(precision(coordinates.find_midpoint()[2], 6)), "1"],
        ["3", "0", "3", "21", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.originalCoords()[0], 8)), to_float(precision(coordinates.originalCoords()[1], 8)), to_float(precision(coordinates.originalCoords()[2], 6)), "1"]
    ]

    upload_mission_from_array(waypoints )

