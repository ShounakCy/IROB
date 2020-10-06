#!/usr/bin/env python3

"""
    # {Jinisha Bhanushali}
    # {19940529T668}
    # {jinisha@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

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
        """Updates the grid_map with the data from the laser scan and the pose.
       
        For E:
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """
        map_x =[]
        map_y = []
        ind_x = []
        ind_y = []      
        robot_x = []
        robot_y = []
        px=pose.pose.position.x
        py=pose.pose.position.y
       
     
        #For occupied space
        i=0
        for j in range(len( (scan.ranges))):
           
            if scan.range_min >=  (scan.ranges[j]) or  (scan.ranges[j]) >= scan.range_max:
                     continue

            else:
                b_angle = scan.angle_min + (j * scan.angle_increment)
           
                map_x.append(px +  (scan.ranges[j]) * cos(robot_yaw + b_angle))
                map_y.append(py +  (scan.ranges[j]) * sin(robot_yaw + b_angle))

                ind_x.append(int((map_x[i]- origin.position.x) / resolution))
                ind_y.append(int((map_y[i]- origin.position.y) / resolution))

                # grid index of the robot
                robot_x.append(int((px - origin.position.x) / resolution) )
                robot_y.append(int((py - origin.position.y) / resolution))    
                start =(robot_x[i],robot_y[i])
                end = (ind_x[i],ind_y[i])
               
                trace=self.raytrace(start,end)
                i=i+1

                #Filling the free space
                for l in trace:
                    self.add_to_map(grid_map, l[0], l[1], self.free_space)              

        #Filling the occupied space      
        for k in range(len(ind_x)):
            self.add_to_map(grid_map, ind_x[k], ind_y[k], self.occupied_space)


        """
        For C only!
        Fill in the update correctly below.
        """
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = np.amin(ind_x)
        # The minimum y index in 'grid_map' that has been updated
        update.y = np.amin(ind_y)
        # Maximum x index - minimum x index + 1
        update.w = np.amax(ind_x) - np.amin(ind_x) + 1
        # Maximum y index - minimum y index + 1
        update.h = np.amax(ind_y) - np.amin(ind_y) + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        for y_u in range(np.amin(ind_y), np.amax(ind_y) + 1):
            for x_u in range(np.amin(ind_x), np.amax(ind_x) + 1):
                updated_value = grid_map[x_u,y_u]
                update.data.append(updated_value)

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):    
        h = grid_map.get_height()
        w = grid_map.get_width()
       
        for x_0 in range(w):
            for y_0 in range(h):
                if grid_map[x_0,y_0] == self.occupied_space:
                    temp_x = x_0 - self.radius
                    temp_y = y_0 - self.radius
                    for x in range(2 * self.radius+1):
                        for y in range(2 * self.radius+1):
                            distance = (temp_x + x - x_0)**2 +(temp_y + y - y_0)**2
                            if distance <= self.radius **2 and grid_map[temp_x + x,temp_y + y] != self.occupied_space and self.is_in_bounds(grid_map,temp_x + x,temp_y + y):
                                self.add_to_map(grid_map,temp_x + x,temp_y + y,self.c_space)
                            else:
                                pass

        # Return the inflated map
        return grid_map

	
	
	
