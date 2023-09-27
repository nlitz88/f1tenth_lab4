
import unittest
from gap_follow_utils import *

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
        # Test when the difference between a and b exceeds the disparity threshold
        a = 5.0
        b = 8.0
        disparity_threshold = 2.0
        result = is_disparity(a, b, disparity_threshold)
        self.assertTrue(result)

    def test_disparity_false(self):
        # Test when the difference between a and b does not exceed the disparity threshold
        a = 5.0
        b = 5.5
        disparity_threshold = 1.0
        result = is_disparity(a, b, disparity_threshold)
        self.assertFalse(result)

    def test_disparity_equal_values(self):
        # Test when a and b are equal, they should not be considered a disparity
        a = 3.0
        b = 3.0
        disparity_threshold = 1.0
        result = is_disparity(a, b, disparity_threshold)
        self.assertFalse(result)

    def test_disparity_zero_threshold(self):
        # Test when the disparity threshold is set to zero, any difference should be considered a disparity
        a = 5.0
        b = 6.0
        disparity_threshold = 0.0
        result = is_disparity(a, b, disparity_threshold)
        self.assertTrue(result)

class TestGetDisparity(unittest.TestCase):

    def test_right_disparity(self):
        # Test when a < b and the difference exceeds the disparity threshold
        a = 5.0
        b = 8.0
        disparity_threshold = 2.0
        result = get_disparity(a, b, disparity_threshold)
        self.assertEqual(result, DisparityDirection.RIGHT)

    def test_left_disparity(self):
        # Test when a > b and the difference exceeds the disparity threshold
        a = 8.0
        b = 5.0
        disparity_threshold = 2.0
        result = get_disparity(a, b, disparity_threshold)
        self.assertEqual(result, DisparityDirection.LEFT)

    def test_no_disparity(self):
        # Test when the difference between a and b is below the disparity threshold
        a = 5.0
        b = 5.5
        disparity_threshold = 1.0
        result = get_disparity(a, b, disparity_threshold)
        self.assertEqual(result, DisparityDirection.NO_DISPARITY)

    def test_default_no_disparity(self):
        # Test when the function should return NO_DISPARITY by default (unexpected case)
        a = 3.0
        b = 3.0
        disparity_threshold = 1.0
        result = get_disparity(a, b, disparity_threshold)
        self.assertEqual(result, DisparityDirection.NO_DISPARITY)

if __name__ == "__main__":
    unittest.main()