"""Python utility module containing common lidar utility functions.
"""
import math
import numpy as np

from sensor_msgs.msg import LaserScan

def angle_in_range(angle_rad: float, laser_scan: LaserScan) -> bool:
    """Returns whether or not the provided angle is within the minimum and
    maximum angle of the the given laser_scan message.

    Args:
        angle_rad (float): The angle to check in radians.
        laser_scan (LaserScan): The laserscan message containing the minimum and
        maximum angle of that laserscan.

    Returns:
        bool: True if angle_rad is within the possible angles, False if not.
    """
    if angle_rad > laser_scan.angle_max or angle_rad < laser_scan.angle_min:
        return False
    return True

def get_index_from_angle(angle_rad: float, laser_scan: LaserScan) -> int:
    """Returns the approximate index in the laser_scan's ranges array that
    corresponds to the provided angle.

    Args:
        angle_rad (float): Angle (in radians) to get index of in ranges array.
        laser_scan (LaserScan): LaserScan message from LiDAR.

    Raises:
        Exception: Throws exception if the provdided angle isn't within the
        range specified in the LaserScan message.

    Returns:
        int: The index in the ranges array corresponding to the provided angle.
    """

    # 1. First, check to make sure the angle provided is within the boundaries
    #    of the laser scan itself.
    if not angle_in_range(angle_rad=angle_rad, laser_scan=laser_scan):
        raise Exception(f"Provided angle {angle_rad:.4f} ({math.degrees(angle_rad):.4f}) is outside of the range within the provided laserscan message: [{laser_scan.angle_min:.4f} : {laser_scan.angle_max:.4f}]")
    # 2. Multiply the provided angle by the provided angle_increment.
    unadjusted_angle_index = angle_rad*laser_scan.angle_increment
    # 3. Add half the number of indicies in our ranges array to the unadjusted
    #    index.
    ranges_list = laser_scan.ranges
    middle_index = np.floor(len(ranges_list) / 2)
    adjusted_angle_index = unadjusted_angle_index + middle_index
    min_index = 0
    max_index = len(ranges_list) - 1
    angle_index = np.clip(a=adjusted_angle_index, a_min=min_index, a_max=max_index)
    return angle_index

def get_range_from_angle(angle_rad: float, laser_scan: LaserScan) -> float:
    pass

def get_angle_from_index():
    pass