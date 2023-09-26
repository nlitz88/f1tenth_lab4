"""Python utility module containing common lidar utility functions.
"""
from dataclasses import dataclass
import math
from typing import List, Tuple
import numpy as np

from sensor_msgs.msg import LaserScan

def angle_in_range(angle_rad: float, angle_min_rad: float, angle_max_rad: float) -> bool:
    """Returns whether or not the provided angle is within the minimum and
    maximum angle of the the given laser_scan message.

    Args:
        angle_rad (float): The angle to check in radians.
        angle_min_rad (float): The minimum (smallest) angle measured by the
        LiDAR from the scan at hand.
        angle_max_rad (float): The maximum (most positive) angle measured by the
        LiDAR from the scan at hand.

    Returns:
        bool: True if angle_rad is within the possible angles, False if not. 
    """
    if angle_rad > angle_max_rad or angle_rad < angle_min_rad:
        return False
    return True

def get_index_from_angle(angle_rad: float, 
                         angle_min_rad: float, 
                         angle_max_rad: float,
                         angle_increment_rad: float, 
                         ranges_m: List[float]) -> int:
    """Returns the approximate index in the laser_scan's ranges array that
    corresponds to the provided angle.

    Args:
        angle_rad (float): Angle (in radians) to get index of in ranges array.
        angle_min_rad (float): The minimum (smallest) angle measured by the
        LiDAR from the scan at hand.
        angle_max_rad (float): The maximum (most positive) angle measured by the
        LiDAR from the scan at hand.
        angle_increment_rad (float): The increment (in radians) between each
        range within the LiDAR scan's ranges array.
        ranges_m (List[float]): The array of range values that comes from the
        LaserScan message at hand.

    Returns:
        int: The index in the ranges array corresponding to the provided angle.
    """
    # 1. First, check to make sure the angle provided is within the boundaries
    #    of the laser scan itself.
    if not angle_in_range(angle_rad=angle_rad, angle_min_rad=angle_min_rad, angle_max_rad=angle_max_rad):
        raise Exception(f"Provided angle {angle_rad:.4f} ({math.degrees(angle_rad):.4f}) is outside of the range within the provided laserscan message: [{angle_min_rad:.4f} : {angle_max_rad:.4f}]")
    # 2. Multiply the provided angle by the provided angle_increment.
    unadjusted_angle_index = angle_rad*angle_increment_rad
    # 3. Add half the number of indicies in our ranges array to the unadjusted
    #    index.
    middle_index = np.floor(len(ranges_m) / 2)
    adjusted_angle_index = unadjusted_angle_index + middle_index
    min_index = 0
    max_index = len(ranges_m) - 1
    angle_index = np.clip(a=adjusted_angle_index, a_min=min_index, a_max=max_index)
    return angle_index

class IndexRange:
    """Dataclass like class dedicated to maintaining the first and last index in
    a particular range. Primarily used to provide a convenient interface for
    iterating over an index range derived from a ranges array.
    """

    def __init__(self, starting_index: int, ending_index: int):
        self.starting_index: int
        self.ending_index: int
        # Set step for iterator based on if ending index > or < the starting
        # index.
        if starting_index < ending_index:
            self.__step = 1
        else:
            self.__step = -1

    def __iter__(self):
        for i in range(__start=self.starting_index, __stop=self.ending_index, __step=self.__step):
            yield i

def get_index_range_from_angles(start_angle_rad: float, end_angle_rad: float, laser_scan: LaserScan) -> IndexRange:
    """Returns an IndexRange containing the starting and ending index of the
    values in the laser_scan's ranges array that correspond to the provided
    starting and ending angle, respectively. NOTE that it would make the most
    sense for the provided start_angle to the be the lesser or more-negative
    angle, where the end_angle is always greater than the start_angle. This way,
    the indices returned are can be traversed in order in the corresponding
    array.

    Args:
        start_angle_rad (float): The first angle (in radians) you want the index
        of.
        end_angle_rad (float): The second angle (in radians) you want the index
        of. Should be greater than the start angle.
        laser_scan (LaserScan): The LaserScan containing the ranges you want the
        indices from.

    Returns:
        IndexRange: An IndexRange dataclass instance containing the starting and
        ending index.
    """
    start_index = get_index_from_angle(angle_rad=start_angle_rad, laser_scan=laser_scan)
    end_index = get_index_from_angle(angle_rad=end_angle_rad, laser_scan=laser_scan)
    return IndexRange(starting_index=start_index, ending_index=end_index)

def get_range_from_angle(angle_rad: float, laser_scan: LaserScan) -> float:
    """Function that returns the range value at the index in the ranges array
    that approximately corresponds to the provided angle.

    Args:
        angle_rad (float): Angle (in radians) to get index of in ranges array.
        laser_scan (LaserScan): LaserScan message from LiDAR.

    Returns:
        float: The range that corresponds to the provided angle.
    """
    return laser_scan.ranges[get_index_from_angle(angle_rad=angle_rad, laser_scan=laser_scan)]