"""Python utility module containing functions made specifically for the
gap_follow node.
"""

from enum import Enum
from typing import List

import numpy as np
from lidarutils import IndexRange, get_angle_from_index, get_index_from_angle

# NOTE: If I can't get this arclength function to work (for whatever reason),
# could always just re-implement a naive version of it that simply returns a
# fixed value, like something we could parameterize. In fact, I could probably
# try to test the whole pipeline first with a fixed number--that could work too. 

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
        length. Could also call this the arc length expressed as the number of
        consecutive range values that create that arc.
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
    start_index = 0
    start_angle_rad = get_angle_from_index(index=start_index, 
                                       angle_increment_rad=angle_increment_rad, 
                                       angle_min_rad=angle_min_rad)
    end_angle_rad = start_angle_rad + theta_rad
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

class DisparityDirection(Enum):
    LEFT=1
    RIGHT=2
    NO_DISPARITY=3

def get_disparity(a: float, b: float, disparity_threshold_m: float) -> DisparityDirection:
    """Determines whether or not there is a sufficiently large disparity between
    a and b, and if there is, returns the direction that disparity is in. 
    
    a is assumed to come before b in the array they come from. If a is less than
    b, and their difference is >= threshold, then the direction of their
    disparity is to the right (I.e., the value increases as you move to the
    right through the source array). Vice versa for the LEFT disparities.

    Args:
        a (float): The value that comes first (lower index) in the source array.
        b (float): The value that comes second (a's index + 1) in the source
        array.
        disparity_threshold_m (float): The threshold (in meters) for what the
        difference in distance must be between ranges a and b in order for their
        difference to be considered a disparity.

    Returns:
        DisparityDirection: LEFT for a left disparity (a>>b), RIGHT
        for a right disparity (a<<b), and NO_DISPARITY for a disparity that
        isn't large enough.
    """
    disparity = a - b
    if abs(disparity) < disparity_threshold_m:
        return DisparityDirection.NO_DISPARITY
    if disparity < 0:
        # Ex: [1,1,1,2,8,8,8,2,3,1,1,1]. Disparity 2-8 == -6 --> 2 gets extended
        # to the right.
        return DisparityDirection.RIGHT
    if disparity > 0:
        return DisparityDirection.LEFT
    # While this case should never be executed, return NO_DISPARITY by default
    # just in case.
    return DisparityDirection.NO_DISPARITY

def extend_range_value_right(ranges: List[float],
                             starting_index: int, 
                             spaces_to_extend: int) -> None:
    """Extend the value in ranges at starting index to the spaces_to_extend
    values to its right.

    Args:
        ranges (List[float]): Array of range values.
        starting_index (int): Index of value that will be copied to
        spaces_to_extend entries to the right.
        spaces_to_extend (int): The number of spaces to the right of the
        starting index to be assigned the value at the starting_index.
    """
    # Check if the provided starting index is out of bounds.
    if starting_index > len(ranges) - 1 or starting_index < 0:
        raise Exception(f"Provided starting_index == {starting_index} is out of bounds of the ranges array (length == {len(ranges)} == [0, {len(ranges)-1}]) ")
    # Also, ensure that the provided number of spaces_to_extend is non-negative.
    if spaces_to_extend < 0:
        raise Exception(f"Provided number of spaces to extend value at index {starting_index} of ranges array is negative! (spaces_to_extend=={spaces_to_extend})")
   # If not, get the value at that index as the extension value.
    extension_value = ranges[starting_index]
    # Start the iteration at one past the current index.
    current_index = starting_index + 1
    while current_index < len(ranges) and spaces_to_extend > 0:
        ranges[current_index] = extension_value
        current_index += 1
        spaces_to_extend -= 1

def extend_range_value_left(ranges: List[float],
                            starting_index: int, 
                            spaces_to_extend: int) -> None:
    """Extend the value in ranges at starting index to the spaces_to_extend
    values to its left.

    Args:
        ranges (List[float]): Array of range values.
        starting_index (int): Index of value that will be copied to
        spaces_to_extend entries to the left.
        spaces_to_extend (int): The number of spaces to the left of the
        starting index to be assigned the value at the starting_index.
    """
    # Check if the provided starting index is out of bounds.
    if starting_index > len(ranges) - 1 or starting_index < 0:
        raise Exception(f"Provided starting_index == {starting_index} is out of bounds of the ranges array (length == {len(ranges)} == [0, {len(ranges)-1}]) ")
    # Also, ensure that the provided number of spaces_to_extend is non-negative.
    if spaces_to_extend < 0:
        raise Exception(f"Provided number of spaces to extend value at index {starting_index} of ranges array is negative! (spaces_to_extend=={spaces_to_extend})")
    # If not, get the value at that index as the extension value.
    extension_value = ranges[starting_index]
    current_index = starting_index - 1
    while current_index >= 0 and spaces_to_extend > 0:
        ranges[current_index] = extension_value
        current_index -= 1
        spaces_to_extend -= 1

    # TODO: STILL NEED TO UNIT TEST THE HELL OUT OF THIS FUNCTION!!!

# Function to get disparities. What should this return/do? I could have it find
# disparities (indices whose value difference meets or exceeds the threshold),
# and then could extend those. 

# Could loop through the provided ranges and run a check "is_disparity" and then
# decide to extend if so. Will need the is_disparity function anyway for a pair
# of values, so write that here regardless.

def pad_disparities(ranges: List[float],
                    angle_increment_rad: float,
                    angle_min_rad: float,
                    angle_max_rad: float,
                    range_indices: List[int],
                    disparity_threshold_m: float,
                    car_width_m: float) -> None:
    """Finds disparities in ranges array and "pads them" with the smaller value
    of the disparity such that the resulting gaps in the ranges array are known
    to be passable.

    Args:
        ranges (List[float]): Array of range values from LaserScan.
        angle_increment_rad (float): Angle increment )in radians) between each
        LiDAR range value.
        angle_min_rad (float): The minimum (smallest) angle measured by the
        LiDAR from the scan at hand.
        angle_max_rad (float): The maximum (most positive) angle measured by the
        LiDAR from the scan at hand.
        range_indices (List[int]): The beginning and ending indices of the
        portion of ranges that disparities should be padded in.
        disparity_threshold_m (float): The minimum distance between two
        consecutive/contiguous range values for their difference to be
        considered a disparity.
        car_width_m (float): The width of the car in meters.
    """

    start_index = range_indices[0]
    stop_index = range_indices[-1]
    # Iterate through each pair of values in the ranges array. Evaluate whether
    # or not each pair is a disparity or not.
    for left in range(start_index, stop_index - 2):
        right = left + 1

        left_range = ranges[left]
        right_range = ranges[right]
        
        disparity = get_disparity(a=left_range,
                                  b=right_range,
                                  disparity_threshold_m=disparity_threshold_m)
        
        if disparity == DisparityDirection.RIGHT:

            # Compute the number of indices/spaces to extend based on the
            # car width and the range (==depth==distance) the shorter value
            # that disparity occurs at.
            arc_length_indices = get_arclength_index_count(radius_m=left_range,
                                                           desired_arc_length_m=0.5*car_width_m,
                                                           angle_increment_rad=angle_increment_rad,
                                                           angle_min_rad=angle_min_rad,
                                                           angle_max_rad=angle_max_rad,
                                                           num_ranges=len(ranges))
            # NOTE: Do we have to subtract one from the arc length to extend by,
            # as the arc length includes the starting index?
            extend_range_value_right(ranges=ranges, 
                                     starting_index=left,
                                     spaces_to_extend=arc_length_indices - 1)
        
        elif disparity == DisparityDirection.LEFT:
            
            # Compute the number of indices/spaces to extend based on the
            # car width and the range (==depth==distance) the shorter value
            # that disparity occurs at.
            arc_length_indices = get_arclength_index_count(radius_m=right_range,
                                                           desired_arc_length_m=0.5*car_width_m,
                                                           angle_increment_rad=angle_increment_rad,
                                                           angle_min_rad=angle_min_rad,
                                                           angle_max_rad=angle_max_rad,
                                                           num_ranges=len(ranges))
            # NOTE: Do we have to subtract one from the arc length to extend by,
            # as the arc length includes the starting index?
            extend_range_value_left(ranges=ranges,
                                    starting_index=right,
                                    spaces_to_extend=arc_length_indices)
        



        