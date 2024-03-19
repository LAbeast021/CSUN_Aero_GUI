# from precision_function import precision
from pymavlink import mavutil
from pymavlink import mavwp
from time import sleep

from waypoint_generator import waypointGenerator

master = mavutil.mavlink_connection('/dev/tty.usbserial-B0016NGB', baud=57600)
master.wait_heartbeat()


def upload_mission_from_array(waypoints):
    master.waypoint_clear_all_send()  # Clear existing mission
    sleep(2) 

    master.waypoint_count_send(len(waypoints))
    for waypoint in waypoints:
        # Convert waypoint data to the expected types
        seq, current, frame, command = int(waypoint[0]), int(waypoint[1]), int(waypoint[2]), int(waypoint[3])
        param1, param2, param3, param4 = float(waypoint[4]), float(waypoint[5]), float(waypoint[6]), float(waypoint[7])
        x, y, z = float(waypoint[8]), float(waypoint[9]), float(waypoint[10])
        autocontinue = int(waypoint[11])

        # Wait for the vehicle to request each waypoint
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True)
        print(f"Vehicle requested waypoint {msg.seq}")

        # Create and send the mission item
        master.mav.mission_item_send(master.target_system, master.target_component,
                                    seq, frame, command,
                                    current, autocontinue, param1, param2, param3, param4,
                                    x, y, z)
        print(f"Sending waypoint {seq}: Lat {x}, Lon {y}, Alt {z}")

    # Wait for mission acknowledgment
    ack = master.recv_match(type='MISSION_ACK', blocking=True)
    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission successfully uploaded.")
    else:
        print(f"Mission upload failed with error: {ack.type}")
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
    coordinates = waypointGenerator(lat, lon, 0 ,direction_code)

    # Generate waypoints in arrays
    waypoints = [
        ["0", "1", "3", "16", "0", "0", "0", "0", to_float(precision(coordinates.waypoint()[0], 7)), to_float(precision(coordinates.waypoint()[1], 7)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["1", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.waypoint()[0], 8)), to_float(precision(coordinates.waypoint()[1], 8)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["2", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.find_midpoint()[0], 8)), to_float(precision(coordinates.find_midpoint()[1], 8)), to_float(precision(coordinates.find_midpoint()[2], 6)), "1"],
        ["3", "0", "3", "21", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.originalCoords()[0], 8)), to_float(precision(coordinates.originalCoords()[1], 8)), to_float(precision(coordinates.originalCoords()[2], 6)), "1"]
    ]

    upload_mission_from_array(waypoints)


