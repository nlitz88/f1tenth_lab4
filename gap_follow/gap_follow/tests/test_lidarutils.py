
import unittest
from lidarutils import *

class TestIndexRange(unittest.TestCase):
    
    def setUp(self) -> None:
        return super().setUp()
    
    def test_iter(self):
        pass

    def tearDown(self) -> None:
        return super().tearDown()
    
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

if __name__ == "__main__":
    unittest.main()