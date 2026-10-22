"""Run with: source /opt/ros/jazzy/setup.bash && python3 -m unittest discover -s tools -p test_record_rosbag.py"""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import unittest.mock

WORKSPACE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("finalizer", WORKSPACE / "tools/finalize_rosbag.py")
finalizer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(finalizer)

# Use the installed ROS MCAP writer without DDS/network or robot topics. This
# process inherits the same background-command signal dispositions as ros2.
RECORDER = '''#!/usr/bin/env python3
import os, pathlib, signal, sys, time
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String
if sys.argv[1:3] == ['topic', 'pub']:
    sys.exit(0)
assert signal.getsignal(signal.SIGINT) != signal.SIG_IGN
if os.environ.get('FAIL_RECORD'):
    sys.exit(7)
assert '--disable-keyboard-controls' in sys.argv
writer = rosbag2_py.SequentialWriter()
directory = sys.argv[sys.argv.index('-o') + 1]
config = sys.argv[sys.argv.index('--storage-config-file') + 1]
writer.open(rosbag2_py.StorageOptions(uri=directory, storage_id='mcap', storage_config_uri=config),
            rosbag2_py.ConverterOptions('', ''))
writer.create_topic(rosbag2_py.TopicMetadata(id=0, name='/test', type='std_msgs/msg/String', serialization_format='cdr'))
writer.write('/test', serialize_message(String(data='test' * 10000)), 1000000000)
def stop(signum, frame):
    global writer
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    time.sleep(0.4)  # Make a second Ctrl+C arrive during index finalization.
    del writer
    pathlib.Path(directory, 'closed').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
pathlib.Path(directory, 'ready').touch()
while True:
    time.sleep(0.05)
'''


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        cli = self.root / "ros2"
        cli.write_text(RECORDER)
        cli.chmod(0o755)
        self.env = dict(os.environ, PATH=f"{self.root}:{os.environ['PATH']}",
                        ROSBAG_DIR=str(self.root / "bags"))

    def tearDown(self):
        self.temp.cleanup()

    def test_stop_signals_flush_indexes(self):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=sig):
                name = f"signal_{sig}"
                command = ('source "$1"; trap stop_recording INT TERM HUP; '
                           'start_recording "$2"; exit "$RECORD_STATUS"')
                with subprocess.Popen(
                    ['bash', '-c', command, 'test',
                     str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh'), name],
                    env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                ) as proc:
                    directory = self.root / 'bags' / name
                    try:
                        deadline = time.monotonic() + 10
                        while not (directory / 'ready').exists():
                            if proc.poll() is not None or time.monotonic() > deadline:
                                self.fail('Recorder failed to start')
                            time.sleep(0.02)
                        proc.send_signal(sig)
                        time.sleep(0.1)
                        proc.send_signal(signal.SIGINT)
                        output = proc.communicate(timeout=15)[0].decode()
                        self.assertEqual(proc.returncode, 0, output)
                        self.assertTrue((directory / 'closed').exists(), output)
                        paths = list(directory.glob('*.mcap'))
                        self.assertEqual(len(paths), 1)
                        self.assertTrue(finalizer.is_finalized(paths[0]), output)
                        self.assertIn('Foxgloveで開けます', output)
                    finally:
                        if proc.poll() is None:
                            proc.kill()

    def test_failed_recorder_is_not_reported_as_success(self):
        command = 'source "$1"; start_recording failed; exit "$RECORD_STATUS"'
        result = subprocess.run(
            ['bash', '-c', command, 'test', str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh')],
            env=dict(self.env, FAIL_RECORD='1'), capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('Foxgloveで開けます', result.stdout)

    def test_truncated_file_is_detected_and_original_retained(self):
        directory = self.root / 'damaged'
        directory.mkdir()
        path = directory / 'test.mcap'
        path.write_bytes(finalizer.MAGIC + b'truncated record')
        self.assertFalse(finalizer.is_finalized(path))
        with unittest.mock.patch.dict(os.environ, MCAP_BIN='/nonexistent/mcap'):
            with self.assertRaises(RuntimeError):
                finalizer.finalize(directory)
        self.assertEqual(path.read_bytes(), finalizer.MAGIC + b'truncated record')

    @unittest.skipUnless((WORKSPACE / '.runtime/bin/mcap').exists(), 'MCAP CLI not installed')
    def test_recovery_preserves_original_and_rebuilds_metadata(self):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String

        directory = self.root / 'recover'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/test', type='std_msgs/msg/String', serialization_format='cdr'))
        writer.write('/test', serialize_message(String(data='recover me')), 1000000000)
        del writer
        path = next(directory.glob('*.mcap'))
        damaged = path.read_bytes()[:-37]
        path.write_bytes(damaged)
        finalizer.finalize(directory)
        self.assertTrue(finalizer.is_finalized(path))
        self.assertEqual(path.with_suffix('.mcap.damaged').read_bytes(), damaged)
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        self.assertEqual(reader.read_next()[0], '/test')
        self.assertFalse(reader.has_next())


if __name__ == '__main__':
    unittest.main()
