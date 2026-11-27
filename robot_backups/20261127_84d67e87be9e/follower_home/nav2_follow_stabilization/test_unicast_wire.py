import unittest
from unicast_wire import encode, decode, HEADER


class WireTests(unittest.TestCase):
    def test_preserves_original_bytes(self):
        raw = b'original-source-stamp=1234\x00' * 3000
        self.assertEqual(decode(encode(1, 7, 10**10, raw), 10**10), (1, 7, raw))

    def test_stale_and_future_datagrams_are_rejected(self):
        packet = encode(1, 1, 10**10, b'x')
        for now in (10**10 + 300_000_001, 10**10 - 50_000_001):
            with self.assertRaises(ValueError): decode(packet, now)

    def test_invalid_truncated_and_oversize_payload(self):
        packet = encode(1, 1, 10**10, b'x' * 1000)
        for bad in (b'x', packet[:-1], b'BAD!' + packet[4:]):
            with self.assertRaises(ValueError): decode(bad, 10**10)
        with self.assertRaises(ValueError): encode(1, 1, 10**10, b'x' * 2_000_001)

    def test_ros_odometry_original_stamp_is_preserved(self):
        from rclpy.serialization import serialize_message, deserialize_message
        from nav_msgs.msg import Odometry
        msg = Odometry()
        msg.header.stamp.sec = 123
        msg.header.frame_id = 'map'
        raw = bytes(serialize_message(msg))
        payload = decode(encode(1, 1, 10**10, raw), 10**10)[2]
        self.assertEqual(payload, raw)
        self.assertEqual(deserialize_message(payload, Odometry).header.stamp.sec, 123)


if __name__ == '__main__': unittest.main()
