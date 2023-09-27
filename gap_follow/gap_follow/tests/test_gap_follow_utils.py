
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
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [3.0, 3.0, 3.0, 4.0, 5.0])

    def test_extend_left_with_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = 2
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [2.0, 2.0, 3.0, 4.0, 5.0])
    
    def test_extend_left_with_more_clipping(self):
        # Test case 2: Requesting an extension that would step past the start of
        # the array.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 0
        spaces_to_extend = 12
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_extend_full_length(self):
        # Test case 3: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 4
        spaces_to_extend = 4
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [5.0, 5.0, 5.0, 5.0, 5.0])

    def test_extend_partial_length(self):
        # Test case 4: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 4
        spaces_to_extend = 2
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 5.0, 5.0, 5.0])

    def test_extend_zero_length(self):
        # Test case 5: Requesting an extension that would start at the end of
        # the array and fill the entire length.
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 3
        spaces_to_extend = 0
        extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertEqual(ranges, [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_extend_negative_spaces(self):
        # Test case 6: What happens when spaces_to_extend is negative?
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 1
        spaces_to_extend = -3 # Negative number of spaces_to_extend.
        with self.assertRaises(Exception) as context:
            extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertTrue("is negative" in str(context.exception))

    def test_out_of_bounds_exception(self):
        # Test case 7: Ensure out-of-bounds exception is raised
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        starting_index = 10  # Out of bounds index
        spaces_to_extend = 2
        with self.assertRaises(Exception) as context:
            extend_range_value_left(ranges, starting_index, spaces_to_extend)
        self.assertTrue("out of bounds" in str(context.exception))

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

class TestPadDisparities(unittest.TestCase):

    def test_pad_single_right_disparity(self):

        ranges = [1.1, 1.2, 1.0, 1.1, 2.3, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5]
        angle_increment_rad = 0.004351851996034384
        disparity_threshold_m = 0.3
        car_width_m = 0.2032
        pad_disparities(ranges=ranges,
                        angle_increment_rad=angle_increment_rad,
                        range_indices=[0,len(ranges)-1],
                        disparity_threshold_m=disparity_threshold_m,
                        car_width_m=car_width_m)
        expected_result = [1.1, 1.2, 1.0, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5, 2.4, 2.4, 2.3, 2.6, 2.5]
        
        print("Modified ranges:")
        print(ranges)
        print("Expected ranges:")
        print(expected_result)
        self.assertEqual(ranges, expected_result)
    
    def test_pad_single_left_disparity(self):
        ranges = [1.1, 1.2, 1.0, 1.1, 2.3, 2.5, 2.4, 2.4, 2.3, 2.6, 1.3, 1.2, 1.1, 1.0]
        pass

    def test_pad_no_disparity(self):
        pass

    def test_pad_multi_left_disparity(self):
        pass

    def test_pad_multi_right_disparity(self):
        pass
    
    def test_pad_multi_mixed_disparity_1(self):
        pass

    def test_pad_multi_mixed_disparity_2(self):
        pass

if __name__ == "__main__":
    unittest.main()