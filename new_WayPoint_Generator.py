# from precision_function import precision
from waypoint_generator import waypointGenerator

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
def create_waypoints(lat, lon):
    coordinates = waypointGenerator(lat, lon, 0)

    # Generate waypoints in arrays
    waypoints = [
        ["0", "1", "3", "16", "0", "0", "0", "0", to_float(precision(coordinates.waypoint()[0], 7)), to_float(precision(coordinates.waypoint()[1], 7)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["1", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.waypoint()[0], 8)), to_float(precision(coordinates.waypoint()[1], 8)), to_float(precision(coordinates.waypoint()[2], 6)), "1"],
        ["2", "0", "3", "16", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.find_midpoint()[0], 8)), to_float(precision(coordinates.find_midpoint()[1], 8)), to_float(precision(coordinates.find_midpoint()[2], 6)), "1"],
        ["3", "0", "3", "21", "0.00000000", "0.00000000", "0.00000000", "0.00000000", to_float(precision(coordinates.originalCoords()[0], 8)), to_float(precision(coordinates.originalCoords()[1], 8)), to_float(precision(coordinates.originalCoords()[2], 6)), "1"]
    ]


