#!/usr/bin/env python3

"""
    # Akash Singh
    # {student id}
    # akashsin@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        obstacle_list = []
        for i in range(0,len(scan.ranges)):
            if (scan.range_min < scan.ranges[i] < scan.range_max):
                bearing = scan.angle_min + scan.angle_increment*i
                x_map = scan.ranges[i]*cos(bearing + robot_yaw) + pose.pose.position.x - origin.position.x
                y_map = scan.ranges[i]*sin(bearing + robot_yaw) + pose.pose.position.y - origin.position.y
                
                bot_pos_x = pose.pose.position.x - origin.position.x
                bot_pos_y = pose.pose.position.y - origin.position.y

                x_start = int(bot_pos_x/resolution)
                y_start = int(bot_pos_y/resolution)
                                
                x = int(x_map/resolution)
                y = int(y_map/resolution)

                start = [x_start,y_start]
                end = [x, y]

                free_cells = self.raytrace(start, end)
                length_1 = len(free_cells)
                for i in range(0,length_1):
                    #if grid_map[free_cells[i][0], free_cells[i][1]] != self.occupied_space:
                    self.add_to_map(grid_map, free_cells[i][0], free_cells[i][1], self.free_space)
                        
                obstacle_list.append((int(x),int(y)))


        length = len(obstacle_list)
        for i in range(0,length):       
            self.add_to_map(grid_map, obstacle_list[i][0], obstacle_list[i][1], self.occupied_space)

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = 0
        # The minimum y index in 'grid_map' that has been updated
        update.y = 0
        # Maximum x index - minimum x index + 1
        update.width = 0
        # Maximum y index - minimum y index + 1
        update.height = 0

        max_x = 0
        max_y = 0
        x_obs_list = []
        y_obs_list = []

        for i in obstacle_list:
            x_obs_list.append(i[0])
            y_obs_list.append(i[1])

        update.x = min(x_obs_list)
        update.y = min(y_obs_list)
        max_x = max(x_obs_list)
        max_y = max(y_obs_list)

        update.width = max_x - update.x + 1
        update.height = max_y - update.y + 1
        #update.data = [self.unknown_space for _ in range(update.width*update.height)]
        update.data = []
       
        for m in range(0, update.height):
            for n in range(0, update.width):            
                update.data.append(grid_map[n + update.x, m + update.y])

        return grid_map, update

    def inflate_map(self, grid_map):

        radius = self.radius

        for m in range(0, grid_map.get_width()):
            for n in range(0, grid_map.get_width()):            
                if grid_map[m,n] == self.occupied_space:
                    for i in range(-radius, +radius):
                        for j in range(-radius, +radius):
                           x = m+i
                           y = n+j
                           if grid_map[x,y] != self.occupied_space:
                               if (np.sqrt((x-m)**2 + (y-n)**2) <= radius):
                                   self.add_to_map(grid_map, x ,y, self.c_space)
            

        return grid_map
