
import unittest
from gap_follow_utils import *

class TestNumRangesInArcLength(unittest.TestCase):
    
    def test_short_distance(self):
        # Test with short distance from the car to the obstacle.
        distance_m = 0.5
        desired_arc_length_m = 0.1016
        angle_increment_rad = 0.004351851996034384
        expected_result = 47
        result = num_ranges_in_arclength(arc_length_m=desired_arc_length_m,
                                         arc_radius_m=distance_m,
                                         angle_increment_rad=angle_increment_rad)
        self.assertAlmostEqual(result, expected_result, delta=1)

    def test_medium_distance(self):
        # Test with medium distance from the car to the obstacle.
        distance_m = 2.0
        desired_arc_length_m = 0.1016
        angle_increment_rad = 0.004351851996034384
        expected_result = 12
        result = num_ranges_in_arclength(arc_length_m=desired_arc_length_m,
                                         arc_radius_m=distance_m,
                                         angle_increment_rad=angle_increment_rad)
        self.assertAlmostEqual(result, expected_result, delta=1)

    def test_long_distance(self):
        # Test with medium distance from the car to the obstacle.
        distance_m = 4.0
        desired_arc_length_m = 0.1016
        angle_increment_rad = 0.004351851996034384
        expected_result = 6
        result = num_ranges_in_arclength(arc_length_m=desired_arc_length_m,
                                         arc_radius_m=distance_m,
                                         angle_increment_rad=angle_increment_rad)
        self.assertAlmostEqual(result, expected_result, delta=1)

class TestExtendRangeValueLeft(unittest.TestCase):

    def test_extend_left_in_bounds(self):
        # Test case 1: Extending to the left within the bounds of the list
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 2
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [3.0, 3.0, 3.0, 4.0, 5.0])

    def test_extend_left_with_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [2.0, 2.0, 3.0, 4.0, 5.0])
    
    def test_extend_left_with_more_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 0
        spaces_to_extend = 12
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_extend_full_length(self):
        # Test case 3: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 4
        spaces_to_extend = 4
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [5.0, 5.0, 5.0, 5.0, 5.0])

    def test_extend_partial_length(self):
        # Test case 4: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 4
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 5.0, 5.0, 5.0])

    def test_extend_zero_length(self):
        # Test case 5: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 3
        spaces_to_extend = 0
        range_indices = list(range(len(ranges)))
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_extend_negative_spaces(self):
        # Test case 6: What happens when spaces_to_extend is negative?
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = -3 # Negative number of spaces_to_extend.
        range_indices = list(range(len(ranges)))
        with self.assertRaises(Exception) as context:
            extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertTrue("is negative" in str(context.exception))

    def test_out_of_bounds_exception(self):
        # Test case 7: Ensure out-of-bounds exception is raised
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 10  # Out of bounds index
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        with self.assertRaises(Exception) as context:
            extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertTrue("out of bounds" in str(context.exception))

    def test_extend_left_in_bounds_3(self):
        # Test case 8: Testing functionality whenever we're working in only a
        # subset of the values of ranges.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 2.3, 1.9, 3.4, 9.7, 2.3]
        starting_index = 6
        spaces_to_extend = 2
        range_indices = [2,3,4,5,6,7,8]
        extend_range_value_left(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        expected_result = [1.0, 2.0, 3.0, 4.0, 2.3, 2.3, 2.3, 1.9, 3.4, 9.7, 2.3]
        self.assertEqual(ranges, expected_result)

class TestExtendRangeValueRight(unittest.TestCase):

    def test_extend_right_in_bounds(self):
        # Test case 1: Extending to the right within the bounds of the list
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 2
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 3.0, 3.0])

    def test_extend_right_in_bounds_2(self):
        # Similar to 1, but tests extension when num spaces doesn't meet or
        # exceed the length of the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 2.3, 1.9, 3.4, 9.7, 2.3]
        starting_index = 2
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        expected_result = [1.0, 2.0, 3.0, 3.0, 3.0, 4.0, 2.3, 1.9, 3.4, 9.7, 2.3]
        self.assertEqual(ranges, expected_result)

    def test_extend_right_with_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 3
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 4.0])
    
    def test_extend_right_with_more_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 4
        spaces_to_extend = 12
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_extend_full_length(self):
        # Test case 3: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 0
        spaces_to_extend = 4
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 1.0, 1.0, 1.0, 1.0])

    def test_extend_partial_length(self):
        # Test case 4: Requesting an extension
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 2.0, 2.0, 5.0])

    def test_extend_zero_length(self):
        # Test case 5: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 3
        spaces_to_extend = 0
        range_indices = list(range(len(ranges)))
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])
    
    def test_extend_negative_spaces(self):
        # Test case 6: What happens when spaces_to_extend is negative?
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = -3 # Negative number of spaces_to_extend.
        range_indices = list(range(len(ranges)))
        with self.assertRaises(Exception) as context:
            extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertTrue("is negative" in str(context.exception))

    def test_out_of_bounds_exception(self):
        # Test case 7: Ensure out-of-bounds exception is raised
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 10  # Out of bounds index
        spaces_to_extend = 2
        range_indices = list(range(len(ranges)))
        with self.assertRaises(Exception) as context:
            extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        self.assertTrue("out of bounds" in str(context.exception))

    def test_extend_right_in_bounds_3(self):
        # Test case 8: Testing functionality whenever we're working in only a
        # subset of the values of ranges.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 2.3, 1.9, 3.4, 9.7, 2.3]
        starting_index = 3
        spaces_to_extend = 2
        range_indices = [2,3,4,5,6,7,8]
        extend_range_value_right(ranges=ranges, range_indices=range_indices, starting_index=starting_index, spaces_to_extend=spaces_to_extend)
        expected_result = [1.0, 2.0, 3.0, 4.0, 4.0, 4.0, 2.3, 1.9, 3.4, 9.7, 2.3]
        self.assertEqual(ranges, expected_result)

class TestRangesUnderThreshold(unittest.TestCase):

    def setUp(self):
        self.ranges = [1.0, 2.0, 3.0, 0.5, 2.5]
        self.range_indices = [0, 1, 3]
        self.minimum_distance_m = 1.0

    def test_ranges_under_threshold_positive(self):
        # Test when one of the specified ranges falls under the minimum distance threshold.
        result = ranges_under_threshold(self.ranges, self.range_indices, self.minimum_distance_m)
        self.assertTrue(result)

    def test_ranges_under_threshold_negative(self):
        # Test when none of the specified ranges fall under the minimum distance threshold.
        self.minimum_distance_m = 0.1  # Reduce the threshold to make all ranges pass
        result = ranges_under_threshold(self.ranges, self.range_indices, self.minimum_distance_m)
        self.assertFalse(result)

    def test_ranges_under_threshold_empty_indices(self):
        # Test with an empty range_indices list, which should always return False.
        self.range_indices = []
        result = ranges_under_threshold(self.ranges, self.range_indices, self.minimum_distance_m)
        self.assertFalse(result)

class TestIsDisparity(unittest.TestCase):
    def test_disparity_true(self):
        # Test when the difference between a and b is greater than or equal to the disparity threshold.
        a = 10.0
        b = 7.0
        disparity_threshold_m = 2.0
        result = is_disparity(a, b, disparity_threshold_m)
        self.assertTrue(result)

    def test_disparity_false(self):
        # Test when the difference between a and b is less than the disparity threshold.
        a = 5.0
        b = 6.0
        disparity_threshold_m = 2.0
        result = is_disparity(a, b, disparity_threshold_m)
        self.assertFalse(result)

    def test_disparity_equal_threshold(self):
        # Test when the difference between a and b is equal to the disparity threshold.
        a = 8.0
        b = 6.0
        disparity_threshold_m = 2.0
        result = is_disparity(a, b, disparity_threshold_m)
        self.assertTrue(result)

class TestGetDisparityDirection(unittest.TestCase):
    def test_right_disparity(self):
        result = get_disparity_direction(2, 1)
        self.assertEqual(result, DisparityDirection.LEFT)

    def test_left_disparity(self):
        result = get_disparity_direction(1, 2)
        self.assertEqual(result, DisparityDirection.RIGHT)

    def test_no_disparity(self):
        result = get_disparity_direction(2, 2)
        self.assertEqual(result, DisparityDirection.NO_DISPARITY)

    def test_large_right_disparity(self):
        result = get_disparity_direction(10, 2)
        self.assertEqual(result, DisparityDirection.LEFT)

    def test_large_left_disparity(self):
        result = get_disparity_direction(2, 10)
        self.assertEqual(result, DisparityDirection.RIGHT)

class TestFindDisparities(unittest.TestCase):
    
    def test_many_disparities(self):
        # Test case 1: No disparities
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        range_indices = [0, 1, 2, 3, 4]
        disparity_threshold_m = 1.0
        result = find_disparities(ranges, range_indices, disparity_threshold_m)
        self.assertEqual(result, [Disparity(left_index=0, right_index=1, direction=DisparityDirection.RIGHT),
                                  Disparity(left_index=1, right_index=2, direction=DisparityDirection.RIGHT),
                                  Disparity(left_index=2, right_index=3, direction=DisparityDirection.RIGHT),
                                  Disparity(left_index=3, right_index=4, direction=DisparityDirection.RIGHT)])
        
    def test_one_disparity(self):
        # Test case 2: One disparity
        ranges = [1.0, 2.1, 2.5, 2.3]
        range_indices = [0, 1, 2, 3]
        disparity_threshold_m = 1.0
        result = find_disparities(ranges, range_indices, disparity_threshold_m)
        expected_disparity = [Disparity(left_index=0, right_index=1, direction=DisparityDirection.RIGHT)]
        self.assertEqual(result, expected_disparity)

class TestPadDisparities(unittest.TestCase):

    def test_pad_single_right_disparity(self):

        ranges = [1.1, 1.2, 1.0, 1.1, 2.3, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5]
        angle_increment_rad = 0.004351851996034384
        disparity_threshold_m = 0.3
        car_width_m = 0.2032
        # List of disparities from above ranges array.
        disparities = [Disparity(left_index=3, right_index=4, direction=DisparityDirection.RIGHT)]
        pad_disparities(ranges=ranges,
                        range_indices=[1,38],
                        disparities=disparities,
                        car_width_m=car_width_m,
                        angle_increment_rad=angle_increment_rad)
        expected_result = [1.1, 1.2, 1.0, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5]
        self.assertEqual(ranges, expected_result)
    
    def test_pad_single_left_disparity(self):

        ranges = [1.5, 1.5, 1.5, 1.4, 1.5, 1.6, 1.7, 1.5, 1.5, 0.8, 0.75, 0.7, 0.65, 0.5, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45]
        angle_increment_rad = 0.004351851996034384
        disparity_threshold_m = 0.3
        car_width_m = 0.2032
        # List of disparities from above ranges array.
        disparities = [Disparity(left_index=8, right_index=9, direction=DisparityDirection.LEFT)]
        pad_disparities(ranges=ranges,
                        range_indices=[5,38],
                        disparities=disparities,
                        car_width_m=car_width_m,
                        angle_increment_rad=angle_increment_rad)
        expected_result = [1.5, 1.5, 1.5, 1.4, 1.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.75, 0.7, 0.65, 0.5, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45, 0.55, 0.45, 0.45, 0.45, 0.45, 0.45]
        self.assertEqual(ranges, expected_result)
    
    # def test_pad_single_left_disparity(self):
    #     ranges = [1.1, 1.2, 1.0, 1.1, 2.3, 2.5, 2.4, 2.4, 2.3, 2.6, 1.3, 1.2, 1.1, 1.0]
    #     pass

    # def test_pad_no_disparity(self):
    #     pass

    # def test_pad_multi_left_disparity(self):
    #     pass

    # def test_pad_multi_right_disparity(self):
    #     pass
    
    # def test_pad_multi_mixed_disparity_1(self):
    #     pass

    # def test_pad_multi_mixed_disparity_2(self):
    #     pass

class TestFindGaps(unittest.TestCase):

    def test_multiple_gaps(self):
        # Define your test data here, such as ranges and range_indices.
        ranges = [0.5, 0.8, 1.2, 3.0, 0.9, 0.4, 2.5]
        range_indices = [0, 1, 2, 3, 4, 5, 6]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=2, right_index=3),
            Gap(left_index=6, right_index=6)
        ]

        self.assertEqual(gaps, expected_gaps)
    
    def test_single_gap_at_beginning(self):
        # Define your test data here, such as ranges and range_indices.
        ranges = [3.0, 0.8, 0.9, 0.9, 0.9, 0.4, 0.9]
        range_indices = [0, 1, 2, 3, 4, 5, 6]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=0, right_index=0),
        ]
        
        self.assertEqual(gaps, expected_gaps)
    
    def test_single_gap_at_end(self):
        ranges = [0.3, 0.8, 0.9, 0.9, 0.9, 0.4, 2.0]
        range_indices = [0, 1, 2, 3, 4, 5, 6]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=6, right_index=6)
        ]
        
        self.assertEqual(gaps, expected_gaps)

    def test_single_gap_in_middle(self):
        ranges = [0.3, 0.8, 0.9, 1.0, 0.9, 0.4, 0.4]
        range_indices = [0, 1, 2, 3, 4, 5, 6]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=3, right_index=3)
        ]
        
        self.assertEqual(gaps, expected_gaps)


    def test_one_large_gap(self):
        ranges = [1.0, 1.2, 1.0, 1.1, 1.2, 1.3, 1.4]
        range_indices = [0, 1, 2, 3, 4, 5, 6]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=0, right_index=6)
        ]
        
        self.assertEqual(gaps, expected_gaps)

    def test_one_large_gap_in_subrange(self):
        ranges = [0.3, 1.2, 1.0, 1.1, 1.2, 0.9902, 0.4]
        range_indices = [1, 2, 3, 4]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=1, right_index=4)
        ]
        
        self.assertEqual(gaps, expected_gaps)

    def test_gap_beginning_to_middle_in_subrange(self):
        ranges = [0.3, 1.2, 1.0, 0.2, 0.92, 0.9902, 0.4]
        range_indices = [1, 2, 3, 4]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=1, right_index=2)
        ]
        
        self.assertEqual(gaps, expected_gaps)

    def test_gap_middle_to_end_in_subrange(self):
        ranges = [0.3, 0.3, 0.4, 1.2, 1.92, 0.9902, 0.4]
        range_indices = [1, 2, 3, 4]
        gap_distance_threshold_m = 1.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=3, right_index=4)
        ]
        
        self.assertEqual(gaps, expected_gaps)

    def test_multiple_gaps_2_in_subrange(self):
        ranges = [1.1, 1.2, 1.0, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5]
        range_indices = list(range(5, len(ranges)-5))
        gap_distance_threshold_m = 2.0

        # Call the function you want to test
        gaps = find_gaps(ranges, range_indices, gap_distance_threshold_m)

        # Assert the expected results
        expected_gaps = [
            Gap(left_index=26, right_index=35)
        ]
        
        self.assertEqual(gaps, expected_gaps)



if __name__ == "__main__":
    unittest.main()