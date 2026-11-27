import math
import struct
import unittest
from audit_incident_command_chain import inspect_command, Config, MotorCore
from follower_drive_candidate import BoundedDrivePort, DriveContract
from test_motor_candidate import FakeSerial, Clock


class CommandChainTests(unittest.TestCase):
    def setUp(self):
        self.core = MotorCore(config=Config(max_rpm=10))

    def test_straight_motion_and_zero_do_not_create_turn(self):
        for v in (0., .004, .005, .0137, .04, .06):
            r = inspect_command(self.core, v, 0.)
            self.assertEqual(r['raw_rpm'][0], -r['raw_rpm'][1])
            self.assertEqual(r['equivalent_w'], 0.)

    def test_rounding_can_make_one_rpm_difference(self):
        v = 1.5 * 2 * math.pi * .065 / 60
        r = inspect_command(self.core, v, .0001)
        self.assertEqual(r['raw_rpm'], [-1, 2])
        self.assertAlmostEqual(r['equivalent_w'], 2*math.pi*.065/60/.56)

    def test_forward_curve_preserves_turn_direction(self):
        for w in (-.1, .1):
            r = inspect_command(self.core, .06, w)
            self.assertGreater(r['equivalent_w'] * w, 0.)
            self.assertLessEqual(max(map(abs, r['raw_rpm'])), 10)

    def test_dry_run_does_not_require_transport(self):
        self.assertTrue(self.core.dry_run)
        self.assertIsNone(self.core.transport)
        self.core.tick(1_000_000_000)

    def test_m2_alarm_during_motion_sends_zero_and_stays_latched(self):
        clock, transport = Clock(), FakeSerial()
        contract = DriveContract(BoundedDrivePort(transport, clock, max_rpm=10),
            enabled=True, clock=clock, max_rpm=10, trial_seconds=60)
        core = contract.core
        core.tick(clock.ros)
        clock.now += .02
        core.tick(clock.ros)
        contract.zero_samples = 5
        self.assertIsNotNone(contract.arm(clock.ros))
        self.assertTrue(core.accept_command(.04, 0., frame='follower/base_link',
            stamp_ns=clock.ros, ros_now_ns=clock.ros, epoch=core.epoch))
        clock.now += .1
        core.tick(clock.ros)
        self.assertNotEqual(core.last_sent, (0,0))
        # Replace the queued response with a checksum-valid M2 alarm frame.
        data = struct.pack('<hBihBi', 0, 0, 1, 0, 1, 2)
        raw = bytes([0xB8,0xB7,1,216,14]) + data
        transport.rx[:] = raw + bytes([(-sum(raw)) & 255])
        checkpoint = len(transport.sent)
        clock.now += .02
        core.tick(clock.ros)
        self.assertEqual(core.fault, 'DEVICE_STATUS_NONZERO')
        self.assertFalse(core.armed)
        self.assertEqual(core.last_sent, (0,0))
        for _ in range(30):
            clock.now += .02
            core.tick(clock.ros)
        packets = [p for p in transport.sent[checkpoint:] if p[3] == 207]
        self.assertTrue(packets)
        for p in packets:
            self.assertEqual((struct.unpack_from('<h',p,6)[0],
                              struct.unpack_from('<h',p,9)[0]), (0,0))
        # The public driver contract owns fault re-arm policy. MotorCore's
        # legacy explicit-arm method alone is not that external boundary.
        contract.zero_samples = 5
        self.assertIsNone(contract.arm(clock.ros))
        self.assertEqual(core.fault, 'DEVICE_STATUS_NONZERO')


if __name__ == '__main__':
    unittest.main()
