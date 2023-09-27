
import unittest
from gap_follow_utils import *

class TestGetArcLengthIndexCount(unittest.TestCase):

    def test_get_arclength_index_count(self):
        # Define your test cases
        test_cases = [
            {
                'radius_m': 1.0,
                'desired_arc_length_m': 3.14159265359,  # Half circumference of radius 1.0
                'angle_increment_rad': 0.01,
                'angle_min_rad': 0.0,
                'angle_max_rad': 6.28318530718,  # 2 * pi
                'num_ranges': 629,  # Approximately 2 * pi / 0.01
                'expected_result': 316,  # Half of 629
            },
            {
                'radius_m': 2.0,
                'desired_arc_length_m': 6.28318530718,  # Full circumference of radius 2.0
                'angle_increment_rad': 0.01,
                'angle_min_rad': 0.0,
                'angle_max_rad': 6.28318530718,  # 2 * pi
                'num_ranges': 629,  # Approximately 2 * pi / 0.01
                'expected_result': 629,  # Same as num_ranges
            },
            # Add more test cases here
        ]

        # Run the test cases
        for test_case in test_cases:
            result = get_arclength_index_count(
                test_case['radius_m'],
                test_case['desired_arc_length_m'],
                test_case['angle_increment_rad'],
                test_case['angle_min_rad'],
                test_case['angle_max_rad'],
                test_case['num_ranges']
            )
            self.assertAlmostEqual(result, test_case['expected_result'], delta=3)

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

class TestGetDisparityFunction(unittest.TestCase):
    def test_no_disparity(self):
        result = get_disparity(1.0, 1.0, 0.5)
        self.assertEqual(result, DisparityDirection.NO_DISPARITY)

    def test_right_disparity(self):
        result = get_disparity(2.0, 8.0, 5.0)
        self.assertEqual(result, DisparityDirection.RIGHT)

    def test_left_disparity(self):
        result = get_disparity(8.0, 2.0, 5.0)
        self.assertEqual(result, DisparityDirection.LEFT)

    def test_default_case(self):
        result = get_disparity(5.0, 5.0, 0.5)
        self.assertEqual(result, DisparityDirection.NO_DISPARITY)

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