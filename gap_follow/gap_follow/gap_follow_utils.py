"""Python utility module containing functions made specifically for the
gap_follow node.
"""

from dataclasses import dataclass
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

def is_disparity(a: float, b: float, disparity_threshold_m: float) -> bool:
    """Returns whether or not the difference between a and b is >= the provided
    disparity threshold. If it is, then it is considered a disparity, and True
    is returned.

    Args:
        a (float): First value.
        b (float): Second value.
        disparity_threshold_m (float): The threshold distance (in meters) that
        the difference of a-b must be greater than or equal to in order to be
        considered a disparity.

    Returns:
        bool: True if the difference between the first
        and second value meet or exceed the disparity threshold provided
    """
    disparity = a - b
    if abs(disparity) < disparity_threshold_m:
        return False
    return True

def get_disparity_direction(a: float, b: float) -> DisparityDirection:
    """Returns the "direction" of a disparity. Direction refers to the direction
    in which the smaller value of the pair would need to be extended in order to
    pad a disparity.

    For ex: [1,1,1,2,8,8] --> 2 would need to be extended to the right, so we
    call this a "RIGHT" disparity.

    Args:
        a (float): First element.
        b (float): The next contiguous vaule in the array after the first
        element a.

    Returns:
        DisparityDirection: LEFT for a left disparity (a>>b), RIGHT
        for a right disparity (a<<b), and NO_DISPARITY for a disparity that
        isn't large enough or that isn't really a disparity.
    """
    difference = a - b
    if difference < 0:
        return DisparityDirection.RIGHT
    if difference > 0:
        return DisparityDirection.LEFT
    return DisparityDirection.NO_DISPARITY

def extend_range_value_right(ranges: List[float],
                             range_indices: List[int],
                             starting_index: int, 
                             spaces_to_extend: int) -> None:
    """Extend the value in ranges at starting index to the spaces_to_extend
    values to its right.

    Args:
        ranges (List[float]): Array of range values.
        range_indices (List[int]): List of ranges array indices that this
        function will operate within.
        starting_index (int): Index of value that will be copied to
        spaces_to_extend entries to the right.
        spaces_to_extend (int): The number of spaces to the right of the
        starting index to be assigned the value at the starting_index.
    """
    # Grab the boundary indices from the range_indices.
    start = range_indices[0]
    end = range_indices[-1]
    # Check if the provided starting index is out of bounds.
    if starting_index > end or starting_index < start:
        raise Exception(f"Provided starting_index == {starting_index} is out of bounds of the ranges indices of the ranges array provided [{start},{end}].")
    # Also, ensure that the provided number of spaces_to_extend is non-negative.
    if spaces_to_extend < 0:
        raise Exception(f"Provided number of spaces to extend value at index {starting_index} of ranges array is negative! (spaces_to_extend=={spaces_to_extend})")
   # If not, get the value at that index as the extension value.
    extension_value = ranges[starting_index]
    # Start the iteration at one past the current index.
    current_index = starting_index + 1
    while current_index <= end and spaces_to_extend > 0:
        ranges[current_index] = extension_value
        current_index += 1
        spaces_to_extend -= 1

def extend_range_value_left(ranges: List[float],
                            range_indices: List[int],
                            starting_index: int, 
                            spaces_to_extend: int) -> None:
    """Extend the value in ranges at starting index to the spaces_to_extend
    values to its left.

    Args:
        ranges (List[float]): Array of range values.
        range_indices (List[int]): List of ranges array indices that this
        function will operate within.
        starting_index (int): Index of value that will be copied to
        spaces_to_extend entries to the left.
        spaces_to_extend (int): The number of spaces to the left of the
        starting index to be assigned the value at the starting_index.
    """
    # Grab the boundary indices from the range_indices.
    start = range_indices[0]
    end = range_indices[-1]
    # Check if the provided starting index is out of bounds.
    if starting_index > end or starting_index < start:
        raise Exception(f"Provided starting_index == {starting_index} is out of bounds of the ranges indices of the ranges array provided [{start},{end}].")
    # Also, ensure that the provided number of spaces_to_extend is non-negative.
    if spaces_to_extend < 0:
        raise Exception(f"Provided number of spaces to extend value at index {starting_index} of ranges array is negative! (spaces_to_extend=={spaces_to_extend})")
   # If not, get the value at that index as the extension value.
    extension_value = ranges[starting_index]
    current_index = starting_index - 1
    while current_index >= start and spaces_to_extend > 0:
        ranges[current_index] = extension_value
        current_index -= 1
        spaces_to_extend -= 1

# def pad_disparities(ranges: List[float],
#                     angle_increment_rad: float,
#                     range_indices: List[int],
#                     disparity_threshold_m: float,
#                     car_width_m: float) -> None:
#     """Finds disparities in ranges array and "pads them" with the smaller value
#     of the disparity such that the resulting gaps in the ranges array are known
#     to be passable.

#     Args:
#         ranges (List[float]): Array of range values from LaserScan.
#         angle_increment_rad (float): Angle increment )in radians) between each
#         LiDAR range value.
#         range_indices (List[int]): The beginning and ending indices of the
#         portion of ranges that disparities should be padded in.
#         disparity_threshold_m (float): The minimum distance between two
#         consecutive/contiguous range values for their difference to be
#         considered a disparity.
#         car_width_m (float): The width of the car in meters.
#     """

#     start_index = range_indices[0]
#     stop_index = range_indices[-1]
#     # Iterate through each pair of values in the ranges array. Evaluate whether
#     # or not each pair is a disparity or not.
#     for left in range(start_index, stop_index - 2):
#         right = left + 1

#         left_range = ranges[left]
#         right_range = ranges[right]
        
#         disparity = get_disparity(a=left_range,
#                                   b=right_range,
#                                   disparity_threshold_m=disparity_threshold_m)

#         if disparity == DisparityDirection.RIGHT:

#             print(f"Found RIGHT disparity: {left_range}, {right_range}")
#             # Compute the number of indices/spaces to extend based on the
#             # car width and the range (==depth==distance) the shorter value
#             # that disparity occurs at.
#             approx_desired_arc = 0.5*car_width_m
#             num_ranges = num_ranges_in_arclength(arc_length_m=approx_desired_arc,
#                                                  arc_radius_m=left_range,
#                                                  angle_increment_rad=angle_increment_rad)
            
#             print(f"Num ranges in arc length at range {left_range}: {num_ranges}")
#             # The number of indices returned is the total number of indices
#             # needed to create that arc. Therefore, only extend by num - 1, as
#             # there is already a value at the starting_index.
#             print("Ranges before value extension: ")
#             print(ranges)
#             extend_range_value_right(ranges=ranges, 
#                                      starting_index=left,
#                                      spaces_to_extend=num_ranges - 1)

#             # Okay, problem. Basically, when we do one range extension, we end
#             # up creating a new disparity further down the line. And if we're
#             # just extending these one after another, then if we have a gap, for
#             # example, we'd just keep extending until the whole gap is covered
#             # over. So this approach doesn't work.

#             # Instead, we really need to go through the array first one time,
#             # find all the disparities, record their "direction" and which value
#             # we want extended. Then, have another loop that goes back and
#             # performs those extensions after the fact.
            
#             # Therefore, we should break up this function into two separate
#             # functions: 
#             # 1. find_disparities
#             # 2. pad_disparities


        
#         elif disparity == DisparityDirection.LEFT:
            
#             # Compute the number of indices/spaces to extend based on the
#             # car width and the range (==depth==distance) the shorter value
#             # that disparity occurs at.
#             approx_desired_arc = 0.5*car_width_m
#             num_ranges = num_ranges_in_arclength(arc_length_m=approx_desired_arc,
#                                                  arc_radius_m=right_range,
#                                                  angle_increment_rad=angle_increment_rad)
#             # The number of indices returned is the total number of indices
#             # needed to create that arc. Therefore, only extend by num - 1, as
#             # there is already a value at the starting_index.
#             extend_range_value_left(ranges=ranges,
#                                     starting_index=right,
#                                     spaces_to_extend=num_ranges - 1)

@dataclass
class Disparity:
    left_index: int
    right_index: int
    direction: DisparityDirection

def find_disparities(ranges: List[float],
                     range_indices: List[int],
                     disparity_threshold_m: float) -> List[Disparity]:
    """Returns a list of Disparity objects that serve as references to positions
    in the ranges array where a disparity is found.

    Args:
        ranges (List[float]): List of ranges from LaserScan.
        range_indices (List[int]): Indices of working area of ranges.
        disparity_threshold_m (float): The minimum difference between two
        consecutive range values for them to be considered a disparity.

    Returns:
        List[Disparity]: List of Disparity Objects, which are basically just
        references to different locations in the ranges where there are
        disparities.
    """
    
    # Grab start/stop index from range_indices.
    start_index = range_indices[0]
    stop_index = range_indices[-1]
    # Iterate through each pair of values in the ranges array. Evaluate whether
    # or not each pair is a disparity or not. If it is, add it to the below list
    # of Disparity objects.
    disparities = []
    for left in range(start_index, stop_index - 2):
        right = left + 1

        left_range = ranges[left]
        right_range = ranges[right]
        
        # If a disparity is found at the current location, determine its
        # direction and then record this disparity's existence in the
        # disparities list.
        if is_disparity(a=left_range,
                        b=right_range, 
                        disparity_threshold_m=disparity_threshold_m):
            disparity_direction = get_disparity_direction(a=left_range, b=right_range)
            disparities.append(Disparity(left_index=left, right_index=right, direction=disparity_direction))
    return disparities

def pad_disparities(ranges: List[float],
                    range_indices: List[int],
                    disparities: List[Disparity],
                    car_width_m: float,
                    angle_increment_rad: float) -> None:
    """Takes a list of disparities found within ranges and pads each of them
    according to their direction, range value, and the width of the car.
    Modifies the ranges list IN PLACE.

    Args:
        ranges (List[float]): List of ranges from LaserScan.
        range_indices (List[int]): Indices of working area of ranges.
        disparities (List[Disparity]): List of Disparity Objects, which are
        basically just references to different locations in the ranges where
        there are disparities.
        car_width_m (float): The width of the car in meters.
        angle_increment_rad (float): Angle increment (in radians) between each
        LiDAR range value.
    """
    for disparity in disparities:

        if disparity.direction == DisparityDirection.RIGHT:
            # If disparity direction was RIGHT, then compute the number of range
            # values (index values) that can comprise the desired arc.
            approx_desired_arc = 0.5*car_width_m
            num_ranges = num_ranges_in_arclength(arc_length_m=approx_desired_arc,
                                                 arc_radius_m=disparity.left_index,
                                                 angle_increment_rad=angle_increment_rad)
            # Then, extend the range at the left index to the right num_ranges times.
            extend_range_value_right(ranges=ranges, starting_index=disparity.left_index, spaces_to_extend=num_ranges)

        elif disparity.direction == DisparityDirection.LEFT:
            # If disparity direction was LEFT, then compute the number of range
            # values (index values) that can comprise the desired arc.
            approx_desired_arc = 0.5*car_width_m
            num_ranges = num_ranges_in_arclength(arc_length_m=approx_desired_arc,
                                                 arc_radius_m=disparity.left_index,
                                                 angle_increment_rad=angle_increment_rad)
            # Then, extend the range at the right index to the left num_ranges times.
            extend_range_value_left(ranges=ranges, starting_index=disparity.right_index, spaces_to_extend=num_ranges)

    return