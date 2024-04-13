# Application Name: waypoint_generator
# Programmers: Evan Thomas, Daniel Lu
# Last Updated Date:October 3rd, 2022
# Origin Date: September 30th, 2022
# Purpose: To generate two new waypoints based on one landing position
# Moving Forward: Increase precision to 8 decimal points for lat and lon and 6 decimal points for alt
#                 Need to figure out a way to generate a Home Coordinate
# Resources:
# https://www.offroaders.com/technical/understanding-gps-coordinates/
# https://i.stack.imgur.com/Sh97P.jpg
# https://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints#:~:text=The%20basic%20procedure%20is%20as%20follows%3A%201%201.,waypoint%207%207.%20Repeat%20step%206%20until%20complete
# for MAV_CMD_DO_SET_HOME (179) there are 7 parameters https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME
# for MAV_CMD_NAV_WAYPOINT (16) there are 7 parameters (https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT)
# for MAV_CMD_DO_LAND_START (189) 7 parameters total but only two fillable ones (https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START)
# for MAV_CMD_NAV_LAND (21) there are 7 parameters total, but we are only interested in 3, Lat, Lon, Alt (https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND)
# all units are in meters for mavlink, so we need to have a conversion from feet to meters for alt input.

# Plan of attack:
# Program gets the target location
# Program then generates new waypoint relative to target location
# Program then determines the midpoint of the two points

####################################################################################################
# Change Log:
# 2024 Team chnages

# from yolo8_threading_headless import return_coords


class waypointGenerator:
    # allows us to offset gps coordinates in 1ft increments
    lat_conversion = 2.745e-6 #Apollo Field
    lon_conversion = 3.3095732e-6 #Apollo Field

    def __init__(self, lat, lon, alt , direction_code):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.direction_code = direction_code

    # definition that converts ft to meters (might not need in final product)
    def ft_to_m(self, value):
        self.value = value

        meter = value * .3048
        return meter

    def find_midpoint(self):
        waypoint = self.waypoint()
        x1 = self.lat
        y1 = self.lon
        x2 = waypoint[0]
        y2 = waypoint[1]
        mid_x_coordinate = round(((x1 + x2) / 2), 6)
        mid_y_coordinate = round(((y1 + y2) / 2), 6)
        midpoint = [mid_x_coordinate, mid_y_coordinate, self.ft_to_m(26)]
        return midpoint

    # Generate first waypoint:
    def waypoint(self):
# ========================================================================================================================
            # this code creates the waypoint from north -> south
        if self.direction_code == 0:
            waypoint = [round(self.lat + (100 * self.lat_conversion), 6), self.lon, self.ft_to_m(self.alt + 32.81)]
# ========================================================================================================================
            # this code creates the waypoint from south -> north 
        elif self.direction_code == 1 :
            waypoint = [round(self.lat - (100 * self.lat_conversion), 6), self.lon, self.ft_to_m(self.alt + 32.81)]
# ========================================================================================================================
            # this code creates the waypoint from west -> east 
        elif self.direction_code == 2:
            waypoint = [ self.lat ,round(self.lon - (100 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]

# ========================================================================================================================
            # this code creates the waypoint from east -> west
        elif self.direction_code == 3:
            waypoint = [ self.lat ,round(self.lon + (100 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]
# ========================================================================================================================
            # this code creates the waypoint from north east -> south west
        elif self.direction_code == 4:
            waypoint = [ round(self.lat + (141.421356237 * self.lat_conversion), 6) ,round(self.lon + (141.421356237 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]
# ======================================================================================================================== 
            # this code creates the waypoint from south west -> north east
        elif self.direction_code == 5:
            waypoint = [ round(self.lat - (141.421356237 * self.lat_conversion), 6) ,round(self.lon - (141.421356237 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]
# ========================================================================================================================
            # this code creates the waypoint from north west -> south east
        elif self.direction_code == 6:
            waypoint = [ round(self.lat + (141.421356237 * self.lat_conversion), 6) ,round(self.lon - (141.421356237 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]
# ========================================================================================================================
            # this code creates the waypoint from south east -> north west
        elif self.direction_code == 7:
            waypoint = [ round(self.lat - (141.421356237 * self.lat_conversion), 6) ,round(self.lon + (141.421356237 * self.lon_conversion), 6), self.ft_to_m(self.alt + 32.81)]


        return waypoint

    def originalCoords(self):
        coords = [self.lat, self.lon, self.alt]
        return coords

# s = waypointGenerator(32.609974, -97.484244, 0)
# print(s.waypoint())
# print(s.find_midpoint())
# print(s.originalCoords())
