"""Python utility module containing functions made specifically for the
gap_follow node.
"""

from enum import Enum
import math
from typing import List

def num_ranges_in_arclength(arc_length_m: float,
                            arc_radius_m: float,
                            angle_increment_rad: float) -> int:
    """Returns the number of range needed to comprise a an arc with the
    specified arc length at the specified arc radius.

    Args:
        arc_length_m (float): The length of the arc (in meters).
        arc_radius_m (float): The radius/range (in meters) that the arc is
        formed at from as measured from the car.
        angle_increment_rad (float): The increment (in radians) that separates
        each of the range values within the ranges array.

    Returns:
        int: The number of range values that comprise the arc at the given
        distance/radius/range.
    """
    arc_angle_rad = arc_length_m/arc_radius_m
    num_ranges = math.ceil(arc_angle_rad / angle_increment_rad)
    return num_ranges

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
            num_ranges = num_ranges_in_arclength(arc_length_m=0.5*car_width_m,
                                                 arc_radius_m=left_range,
                                                 angle_increment_rad=angle_increment_rad)
            # The number of indices returned is the total number of indices
            # needed to create that arc. Therefore, only extend by num - 1, as
            # there is already a value at the starting_index.
            extend_range_value_right(ranges=ranges, 
                                     starting_index=left,
                                     spaces_to_extend=num_ranges - 1)
        
        elif disparity == DisparityDirection.LEFT:
            
            # Compute the number of indices/spaces to extend based on the
            # car width and the range (==depth==distance) the shorter value
            # that disparity occurs at.
            num_ranges = num_ranges_in_arclength(arc_length_m=0.5*car_width_m,
                                                 arc_radius_m=right_range,
                                                 angle_increment_rad=angle_increment_rad)
            # The number of indices returned is the total number of indices
            # needed to create that arc. Therefore, only extend by num - 1, as
            # there is already a value at the starting_index.
            extend_range_value_left(ranges=ranges,
                                    starting_index=right,
                                    spaces_to_extend=num_ranges - 1)
        



        