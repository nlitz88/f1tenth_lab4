"""Python utility module containing functions made specifically for the
gap_follow node.
"""

from typing import List
from lidarutils import get_angle_from_index, get_index_from_angle

# Generally speaking, need a function that, given the desired circumference
# covered (at a particular given range == radius), should return the indices of
# the angles that go from the starting index to the index of the last angle
# needed to create that circumference.
def get_arclength_index_count(radius_m: float,
                              desired_arc_length_m: float,
                              angle_increment_rad: float,
                              angle_min_rad: float,
                              angle_max_rad: float,
                              num_ranges: int) -> int:
    """Returns the number of indices in the ranges (number of ranges) would be
    needed to create a curve with the desired arc length at the provided radius.

    Args:
        start_index (int): The starting index 
        radius_m (float): The range or radius that the arc would be at.
        desired_arc_length_m (float): Desired length of arc to be formed.
        angle_increment_rad (float): Angle increment )in radians) between each
        LiDAR range value.
        angle_min_rad (float): The minimum (smallest) angle measured by the
        LiDAR from the scan at hand.
        angle_max_rad (float): The maximum (most positive) angle measured by the
        LiDAR from the scan at hand.
        num_ranges (int): The length of the ranges array from the LaserScan
        message. I.e., the number of range measurements.

    Returns:
        int: The number of indices == number of range values from the range
        array that, if plotted, would form an arc of approximately the desired
        length.
    """

    # To compute this, we basically just need to compute what the theta of the
    # curve must be in order to create the desired arclength with at the with
    # the provided radius. If arc_length = r*theta, then we just have to compute
    # theta  = arc_length/radius.
    theta_rad = desired_arc_length_m/radius_m
    # NOTE: Arc length should be padded a little bit, as the arc length doesn't
    # strictly correspond to the width of the gap. That's the side that goes
    # from point to point under the arc. Could compute this if need be, but
    # we'll see if a rough approximation works first.

    # Then, get the angle that corresponds to the provided index, add the
    # computed theta, get the index that corresponds to the resulting angle, and
    # then return an IndexRange from the start_index to the end_index just
    # computed.
    start_angle_rad = get_angle_from_index(index=start_index, 
                                       angle_increment_rad=angle_increment_rad, 
                                       angle_min_rad=angle_min_rad)
    end_angle_rad = start_angle_rad + theta_rad
    start_index = 0
    end_index = get_index_from_angle(angle_rad=end_angle_rad,
                                     angle_min_rad=angle_min_rad,
                                     angle_max_rad=angle_max_rad,
                                     angle_increment_rad=angle_increment_rad,
                                     num_ranges=num_ranges)
    return int(end_index - start_index + 1)

def ranges_under_threshold(ranges: List[float],
                           range_indices: List[int],
                           minimum_distance_m: float) -> bool:
    """Checks whether any of the ranges (corresponding to range_indices)
    fall under the provided minimum_distance threshold. Returns True if any
    of those ranges fall under the minimum threshold, False if not. 

    Args:
        ranges (List[float]): Array of range values from the LaserScan.
        range_indices (List[int]): List of indices of the ranges array. Used
        to select a subset of the range values to evaluate.
        minimum_distance_m (float): The minimum distance all specified
        ranges must be for the function to return False. That is, if any of
        the ranges specified fall under this threshold, the function returns
        True.

    Returns:
        bool: True if any of the ranges fall below the minimum distance
        threshold, False if not.
    """
    for index in range_indices:
        if ranges[index] < minimum_distance_m:
            return True
    return False