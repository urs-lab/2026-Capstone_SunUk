import random
import unittest
from check_clock_offset import bounds


class ClockOffsetTests(unittest.TestCase):
    def test_asymmetric_round_trip_still_bounds_true_offset(self):
        rng=random.Random(20261111)
        for _ in range(1000):
            start=10_000_000_000;offset=rng.randrange(-2_000_000_000,2_000_000_000)
            forward=rng.randrange(1,300_000_000);back=rng.randrange(1,300_000_000)
            low,high=bounds(start,start+forward+offset,start+forward+back)
            self.assertLessEqual(low,offset/1e9);self.assertGreaterEqual(high,offset/1e9)

    def test_local_backward_clock_is_rejected(self):
        with self.assertRaises(ValueError):bounds(10,11,9)
