
import unittest
from lidarutils import *
    
class TestClipRangeValues(unittest.TestCase):

    def test_clip_values(self):
        # Test clipping values within the specified range
        ranges = [0.5, 1.0, 1.5, 2.0, 2.5]
        min_val = 1.0
        max_val = 2.0
        clip_range_values(ranges, min_val, max_val)
        expected_result = [1.0, 1.0, 1.5, 2.0, 2.0]
        self.assertEqual(ranges, expected_result)

    def test_clip_values_with_empty_list(self):
        # Test when the 'ranges' list is empty, it should remain empty
        ranges = []
        min_val = 1.0
        max_val = 2.0
        clip_range_values(ranges, min_val, max_val)
        self.assertEqual(ranges, [])

    def test_clip_values_with_min_max_equal(self):
        # Test when min and max are equal, all values should become that value
        ranges = [0.5, 1.0, 1.5, 2.0, 2.5]
        min_val = 1.5
        max_val = 1.5
        clip_range_values(ranges, min_val, max_val)
        expected_result = [1.5, 1.5, 1.5, 1.5, 1.5]
        self.assertEqual(ranges, expected_result)

class TestGetIndexFromAngle(unittest.TestCase):
    """   "lidar_angle_min_rad": -2.3499999046325684,
            "lidar_angle_max_rad": 2.3499999046325684,
            "lidar_angle_increment_rad": 0.004351851996034384,
            "lidar_num_ranges": 1080

    Args:
        unittest (_type_): _description_
    """
    def test_valid_angle(self):
        angle_rad = 0.0
        angle_increment_rad = 0.004351851996034384
        angle_min_rad = -2.3499999046325684
        angle_max_rad = 2.3499999046325684
        num_ranges = 1080
        expected_index = 539

        result = get_index_from_angle(angle_rad, angle_increment_rad, angle_min_rad, angle_max_rad, num_ranges)

        self.assertEqual(result, expected_index)
    
    def test_max_angle(self):
        angle_rad = 2.3499999046325684
        angle_increment_rad = 0.004351851996034384
        angle_min_rad = -2.3499999046325684
        angle_max_rad = 2.3499999046325684
        num_ranges = 1080
        expected_index = 1079

        result = get_index_from_angle(angle_rad, angle_increment_rad, angle_min_rad, angle_max_rad, num_ranges)

        self.assertEqual(result, expected_index)
    
    def test_min_angle(self):
        angle_rad = -2.3499999046325684
        angle_increment_rad = 0.004351851996034384
        angle_min_rad = -2.3499999046325684
        angle_max_rad = 2.3499999046325684
        num_ranges = 1080
        expected_index = 0

        result = get_index_from_angle(angle_rad, angle_increment_rad, angle_min_rad, angle_max_rad, num_ranges)

        self.assertEqual(result, expected_index)

    def test_angle_out_of_range(self):
        angle_rad = 3.14
        angle_increment_rad = 0.004351851996034384
        angle_min_rad = -2.3499999046325684
        angle_max_rad = 2.3499999046325684
        num_ranges = 1080
        expected_index = 539

        with self.assertRaises(Exception):
            get_index_from_angle(angle_rad, angle_increment_rad, angle_min_rad, angle_max_rad, num_ranges)

class TestGetIndexRangeFromAngles(unittest.TestCase):

    def test_valid_range(self):
        start_angle_rad = -2.3499999046325684
        end_angle_rad = 0.0
        angle_increment_rad = 0.004351851996034384
        angle_min_rad = -2.3499999046325684
        angle_max_rad = 2.3499999046325684
        num_ranges = 1080
        expected_indices = list(range(0, 540))  # Expected indices for the given range
        result = get_index_range_from_angles(start_angle_rad=start_angle_rad,
                                             end_angle_rad=end_angle_rad,
                                             angle_min_rad=angle_min_rad,
                                             angle_max_rad=angle_max_rad,
                                             angle_increment_rad=angle_increment_rad,
                                             num_ranges=num_ranges)
        self.assertEqual(result, expected_indices)

if __name__ == "__main__":
    unittest.main()