"""Run with: source /opt/ros/jazzy/setup.bash && python3 -m unittest discover -s tools -p test_record_rosbag.py"""

import importlib.util
import json
import os
import pty
from pathlib import Path
import signal
import shlex
import struct
import subprocess
import sys
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
assert os.getsid(0) == os.getpid(), 'recorder must have its own session'
writer = rosbag2_py.SequentialWriter()
directory = sys.argv[sys.argv.index('-o') + 1]
config = sys.argv[sys.argv.index('--storage-config-file') + 1]
writer.open(rosbag2_py.StorageOptions(uri=directory, storage_id='mcap', storage_config_uri=config),
            rosbag2_py.ConverterOptions('', ''))
writer.create_topic(rosbag2_py.TopicMetadata(id=0, name='/test', type='std_msgs/msg/String', serialization_format='cdr'))
writer.write('/test', serialize_message(String(data='test' * 10000)), 1000000000)
pathlib.Path(directory, 'arguments.json').write_text(__import__('json').dumps(sys.argv))
def stop(signum, frame):
    global writer
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    time.sleep(float(os.environ.get('FLUSH_DELAY', '0.4')))
    del writer
    pathlib.Path(directory, 'closed').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
pathlib.Path(directory, 'ready').touch()
if os.environ.get('AUTO_EXIT'):
    del writer
    pathlib.Path(directory, 'closed').touch()
    sys.exit(int(os.environ.get('EXIT_STATUS', '0')))
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

    def wait_for_recording(self, proc, name):
        deadline = time.monotonic() + 10
        while not (self.root / 'bags' / name / 'ready').exists():
            self.assertIsNone(proc.poll(), 'Recorder exited before becoming ready')
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.02)

    def assert_saved_without_display(self, name):
        directory = self.root / 'bags' / name
        self.assertTrue((directory / 'closed').exists())
        self.assertTrue(finalizer.is_finalized(next(directory.glob('*.mcap'))))
        self.assertTrue((directory / 'metadata.yaml').exists())
        logs = list((self.root / 'bags/.recording-logs').glob('*.log'))
        self.assertTrue(any('✓ 保存が完了' in p.read_text() for p in logs))

    def test_closed_output_pipe_cannot_interrupt_save(self):
        for sig in (signal.SIGHUP, signal.SIGTERM):
            with self.subTest(signal=sig):
                name = f'closed_pipe_{sig}'
                command = 'source "$1"; trap stop_recording INT TERM HUP; start_recording "$2"; exit "$RECORD_STATUS"'
                with subprocess.Popen(
                        ['bash', '-c', command, 'test',
                         str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh'), name],
                        env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        start_new_session=True) as proc:
                    self.wait_for_recording(proc, name)
                    proc.stdout.close()
                    proc.send_signal(sig)
                    self.assertEqual(proc.wait(timeout=15), 0)
                    self.assert_saved_without_display(name)

    def test_closing_controlling_terminal_still_saves_indexes(self):
        master, slave = pty.openpty()
        bootstrap = '''import os, sys, fcntl, termios
os.setsid()
fcntl.ioctl(0, termios.TIOCSCTTY, 0)
os.execvp(sys.argv[1], sys.argv[1:])
'''
        command = 'source "$1"; trap stop_recording INT TERM HUP; start_recording terminal_close; exit "$RECORD_STATUS"'
        with subprocess.Popen(
                [sys.executable, '-c', bootstrap, 'bash', '-c', command, 'test',
                 str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh')],
                env=self.env, stdin=slave, stdout=slave, stderr=slave) as proc:
            os.close(slave)
            try:
                self.wait_for_recording(proc, 'terminal_close')
                os.close(master)
                master = None
                self.assertEqual(proc.wait(timeout=15), 0)
                self.assert_saved_without_display('terminal_close')
            finally:
                if master is not None:
                    os.close(master)
                if proc.poll() is None:
                    proc.kill()

    def test_launcher_pipeline_survives_its_display_tee_disappearing(self):
        import psutil
        sys.path.insert(0, str(WORKSPACE / 'other_programs/sirius_launcher'))
        from process_manager import ProcessManager

        inner = ('source "$1"; trap stop_recording INT TERM HUP; '
                 'start_recording launcher_pipe; exit "$RECORD_STATUS"')
        rc = self.root / 'launcher.sh'
        display = self.root / 'display.log'
        rc.write_text('bash -c ' + shlex.quote(inner) + ' test '
                      + shlex.quote(str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh'))
                      + ' 2>&1 | tee ' + shlex.quote(str(display)) + '\n')
        with subprocess.Popen(['bash', str(rc)], env=self.env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, start_new_session=True) as proc:
            try:
                self.wait_for_recording(proc, 'launcher_pipe')
                children = psutil.Process(proc.pid).children(recursive=True)
                for child in children:
                    if child.name() == 'tee' and str(display) in child.cmdline():
                        child.kill()  # Terminal/launcher display disappears first.
                manager = ProcessManager('koko_record_rosbag', 'koko_record_rosbag')
                manager.pid_file_content = proc.pid
                manager.pid_file = str(self.root / 'launcher.pid')
                self.assertTrue(manager.stop()['wait_for_save'])
                proc.communicate(timeout=15)
                self.assert_saved_without_display('launcher_pipe')
            finally:
                if proc.poll() is None:
                    proc.kill()

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
                    start_new_session=True,
                ) as proc:
                    directory = self.root / 'bags' / name
                    try:
                        deadline = time.monotonic() + 10
                        while not (directory / 'ready').exists():
                            if proc.poll() is not None or time.monotonic() > deadline:
                                self.fail('Recorder failed to start')
                            time.sleep(0.02)
                        os.killpg(proc.pid, sig)  # Emulate a terminal signal, not just one PID.
                        time.sleep(0.1)
                        for additional in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                            os.killpg(proc.pid, additional)
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
        self.assertIn('起動に失敗', result.stdout)

    def run_recording(self, name, *, mode=None, **environment):
        command = ('source "$1"; '
                   + ('configure_recording_mode "$3"; ' if mode else '')
                   + 'start_recording "$2"; exit "$RECORD_STATUS"')
        return subprocess.run(
            ['bash', '-c', command, 'test', str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh'),
             name, mode or 'real'], env=dict(self.env, AUTO_EXIT='1', **environment),
            capture_output=True, text=True, timeout=15)

    def test_normal_exit_and_abnormal_exit_are_distinguished(self):
        for status in (0, 9):
            with self.subTest(status=status):
                result = self.run_recording(f'exit_[{status}]', EXIT_STATUS=str(status))
                self.assertEqual(result.returncode, status, result.stdout + result.stderr)
                self.assertEqual('✓ 保存が完了' in result.stdout, status == 0)
                self.assertEqual('異常終了' in result.stdout, status != 0)
                self.assertNotIn('保存の確認に失敗', result.stdout)

    def test_save_confirmation_failure_is_distinguished_from_recording_failure(self):
        python = self.root / 'python3'
        python.write_text('''#!/usr/bin/python3
import os, sys
if len(sys.argv) > 1 and sys.argv[1].endswith('/tools/finalize_rosbag.py'):
    sys.exit(23)
os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv[1:])
''')
        python.chmod(0o755)
        result = self.run_recording('failed_confirmation')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('保存の確認に失敗', result.stdout)
        self.assertNotIn('✓ 保存が完了', result.stdout)
        self.assertNotIn('異常終了', result.stdout)
        path = next((self.root / 'bags/failed_confirmation').glob('*.mcap'))
        self.assertTrue(finalizer.is_finalized(path))

    def test_launcher_waits_longer_than_two_seconds_without_killing_writer(self):
        sys.path.insert(0, str(WORKSPACE / 'other_programs/sirius_launcher'))
        from process_manager import ProcessManager

        command = 'source "$1"; trap stop_recording INT TERM HUP; start_recording launcher; exit "$RECORD_STATUS"'
        with subprocess.Popen(
                ['bash', '-c', command, 'test',
                 str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh')],
                env=dict(self.env, FLUSH_DELAY='2.8'), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, start_new_session=True) as proc:
            try:
                deadline = time.monotonic() + 10
                while not (self.root / 'bags/launcher/ready').exists():
                    self.assertIsNone(proc.poll())
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.02)
                manager = ProcessManager('koko_record_rosbag', 'koko_record_rosbag')
                manager.pid_file = str(self.root / 'launcher.pid')
                manager.pid_file_content = proc.pid
                result = manager.stop()
                self.assertTrue(result['wait_for_save'])
                self.assertTrue(manager.is_running())
                time.sleep(2.1)
                manager.force_kill_tree(result['pids'])
                self.assertIsNone(proc.poll(), 'Saving must outlive the old 2-second timeout')
                output = proc.communicate(timeout=10)[0].decode()
                self.assertEqual(proc.returncode, 0, output)
                self.assertIn('✓ 保存が完了', output)
                self.assertFalse(manager.is_running())
            finally:
                if proc.poll() is None:
                    proc.kill()

    def test_simulation_cleanup_requests_save_without_escalating_recorder(self):
        command = 'source "$1"; trap stop_recording INT TERM HUP; start_recording sim_cleanup; exit "$RECORD_STATUS"'
        environment = dict(self.env, FLUSH_DELAY='6.0', ROS_DOMAIN_ID='231',
                           GZ_PARTITION='rosbag_test_only', ROBOTBASE_STOP_SIM_DOMAIN='231',
                           ROBOTBASE_STOP_GZ_PREFIX='rosbag_test_only_',
                           ROBOTBASE_STOP_LAUNCHER_PREFIX=str(self.root / 'launcher_'))
        with subprocess.Popen(
                ['bash', '-c', command, 'test',
                 str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh')],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True) as proc:
            try:
                deadline = time.monotonic() + 10
                while not (self.root / 'bags/sim_cleanup/ready').exists():
                    self.assertIsNone(proc.poll())
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.02)
                result = subprocess.run(
                    ['bash', str(WORKSPACE / 'bash/startup_bash/stop_simulation.sh')],
                    env=environment, capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('保存終了を待っています', result.stderr)
                output = proc.communicate(timeout=15)[0].decode()
                self.assertEqual(proc.returncode, 0, output)
                self.assertIn('✓ 保存が完了', output)
            finally:
                if proc.poll() is None:
                    proc.kill()

    def test_topics_config_and_real_sim_time_options(self):
        for mode in ('real', 'sim'):
            with self.subTest(mode=mode):
                result = self.run_recording(mode, mode=mode)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                args = json.loads((self.root / 'bags' / mode / 'arguments.json').read_text())
                self.assertEqual('--use-sim-time' in args, mode == 'sim')
                self.assertEqual('/clock' in args, mode == 'sim')
                self.assertIn('--include-hidden-topics', args)
                self.assertEqual(args[args.index('--storage') + 1], 'mcap')
                self.assertEqual(args[args.index('--storage-config-file') + 1],
                                 str(WORKSPACE / 'params/common/rosbag_mcap.yaml'))
                topics = {'/rosout', '/cmd_vel_smoothed', '/optimal_trajectory', '/hokuyo_scan',
                          '/experiment_metadata', '/scan3', '/odom', '/odom/filtered', '/imu',
                          '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_direct',
                          '/stop', '/person_area_arrived', '/person_search_done',
                          '/person_search_done_detail', '/person_search/arrival_received',
                          '/parameter_events', '/amcl_pose', '/initialpose', '/map', '/plan',
                          '/global_costmap/costmap', '/local_costmap/costmap',
                          '/local_costmap/published_footprint', '/roboteq/odom',
                          '/cmd_vel_teleop', '/joint_states'}
                topics.update(f'/{action}/_action/{kind}' for action in (
                    'follow_path', 'navigate_to_pose', 'navigate_through_poses', 'spin')
                              for kind in ('status', 'feedback'))
                self.assertTrue(topics.issubset(args))

    def test_interactive_entrypoint_preserves_filename_input_and_mode(self):
        for mode in ('real', 'sim'):
            with self.subTest(mode=mode):
                name = f'entry {mode}'
                result = subprocess.run(
                    ['bash', str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh'), mode],
                    input=f'\n{name}\ntest memo\n', env=dict(self.env, AUTO_EXIT='1'),
                    capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertTrue((self.root / 'bags' / name / 'closed').exists())
                self.assertIn(f'モード: {mode}', result.stdout)
                self.assertIn('実験メモ: test memo', result.stdout)
                self.assertIn('✓ 保存が完了', result.stdout)

    def test_config_writes_zstd_chunks_and_indexes(self):
        result = self.run_recording('compression')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        path = next((self.root / 'bags/compression').glob('*.mcap'))
        records = []
        with path.open('rb') as stream:
            stream.seek(8)
            while stream.tell() < path.stat().st_size - 8:
                opcode, length = struct.unpack('<BQ', stream.read(9))
                payload = stream.read(length)
                records.append(opcode)
                if opcode == 6:
                    size = struct.unpack('<I', payload[28:32])[0]
                    self.assertEqual(payload[32:32 + size], b'zstd')
        for opcode in (6, 7, 8, 2):  # Chunk, MessageIndex, ChunkIndex, Footer.
            self.assertIn(opcode, records)

    def test_signals_during_natural_exit_finalization_do_not_report_early_success(self):
        python = self.root / 'python3'
        python.write_text('''#!/usr/bin/python3
import os, pathlib, sys, time
if len(sys.argv) > 1 and sys.argv[1].endswith('/tools/finalize_rosbag.py'):
    pathlib.Path(sys.argv[2], 'checking').touch()
    time.sleep(0.6)
os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv[1:])
''')
        python.chmod(0o755)
        command = 'source "$1"; start_recording natural; exit "$RECORD_STATUS"'
        with subprocess.Popen(
                ['bash', '-c', command, 'test',
                 str(WORKSPACE / 'bash/startup_bash/record_rosbag.sh')],
                env=dict(self.env, AUTO_EXIT='1'), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, start_new_session=True) as proc:
            try:
                deadline = time.monotonic() + 10
                while not (self.root / 'bags/natural/checking').exists():
                    self.assertIsNone(proc.poll(), 'Recorder exited before validation')
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.02)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                    os.killpg(proc.pid, sig)
                output = proc.communicate(timeout=10)[0].decode()
                self.assertEqual(proc.returncode, 0, output)
                self.assertEqual(output.count('✓ 保存が完了'), 1)
            finally:
                if proc.poll() is None:
                    proc.kill()

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


class FinalizationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def create_bag(self, name):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String

        directory = self.root / name
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name='/test', type='std_msgs/msg/String', serialization_format='cdr'))
        writer.write('/test', serialize_message(String(data=name)), 1000000000)
        del writer
        return directory, next(directory.glob('*.mcap'))

    def test_cli_search_order_and_unusable_override(self):
        runtime = self.root / '.runtime/bin/mcap'
        runtime.parent.mkdir(parents=True)
        path_cli = self.root / 'path/mcap'
        path_cli.parent.mkdir()
        explicit = self.root / 'explicit'
        for cli in (runtime, path_cli, explicit):
            cli.write_text('#!/bin/sh\nexit 0\n')
            cli.chmod(0o755)
        with unittest.mock.patch.object(finalizer, 'WORKSPACE', self.root):
            with unittest.mock.patch.dict(os.environ, MCAP_BIN=str(explicit), PATH=str(path_cli.parent)):
                self.assertEqual(finalizer.find_mcap_cli(), str(explicit))
            with unittest.mock.patch.dict(os.environ, MCAP_BIN='', PATH=str(path_cli.parent)):
                self.assertEqual(finalizer.find_mcap_cli(), str(path_cli))
            with unittest.mock.patch.dict(os.environ, MCAP_BIN='', PATH=''):
                self.assertEqual(finalizer.find_mcap_cli(), str(runtime))
            with unittest.mock.patch.dict(os.environ, MCAP_BIN='/missing/mcap'):
                self.assertRaises(RuntimeError, finalizer.find_mcap_cli)

    def test_existing_backup_and_recovery_output_are_never_overwritten(self):
        for suffix in ('.damaged', '.recovering'):
            with self.subTest(suffix=suffix):
                directory, path = self.create_bag(suffix[1:])
                damaged = path.read_bytes()[:-37]
                path.write_bytes(damaged)
                existing = path.with_name(path.name + suffix)
                existing.write_bytes(b'previous repair must survive')
                with unittest.mock.patch.object(finalizer, 'find_mcap_cli', return_value='/unused'):
                    self.assertRaisesRegex(RuntimeError, '上書きしません', finalizer.finalize, directory)
                self.assertEqual(path.read_bytes(), damaged)
                self.assertEqual(existing.read_bytes(), b'previous repair must survive')

    def test_failed_recovery_retains_original_and_intermediate_output(self):
        for status in (0, 5):
            with self.subTest(status=status):
                directory, path = self.create_bag(f'bad_recover_{status}')
                damaged = path.read_bytes()[:-37]
                path.write_bytes(damaged)

                def bad_recovery(args):
                    Path(args[-1]).write_bytes(b'incomplete recovery')
                    return subprocess.CompletedProcess(args, status)

                with unittest.mock.patch.object(finalizer, 'find_mcap_cli', return_value='/unused'), \
                        unittest.mock.patch.object(finalizer.subprocess, 'run', side_effect=bad_recovery):
                    self.assertRaisesRegex(RuntimeError, '修復に失敗', finalizer.finalize, directory)
                self.assertEqual(path.read_bytes(), damaged)
                self.assertFalse(path.with_name(path.name + '.damaged').exists())
                self.assertEqual(path.with_name(path.name + '.recovering').read_bytes(),
                                 b'incomplete recovery')

    @unittest.skipUnless((WORKSPACE / '.runtime/bin/mcap').exists(), 'MCAP CLI not installed')
    def test_all_split_files_are_checked_and_partial_footer_is_recovered(self):
        directory, first = self.create_bag('split')
        _, second = self.create_bag('second')
        good_first = first.read_bytes()
        target = directory / 'split_1.mcap'
        damaged = second.read_bytes()[:-20]  # Truncation inside the Footer.
        target.write_bytes(damaged)
        finalizer.finalize(directory)
        self.assertEqual(first.read_bytes(), good_first)
        self.assertTrue(finalizer.is_finalized(target))
        self.assertEqual(target.with_suffix('.mcap.damaged').read_bytes(), damaged)
        import yaml
        metadata = yaml.safe_load((directory / 'metadata.yaml').read_text())
        self.assertEqual(metadata['rosbag2_bagfile_information']['message_count'], 2)

    @unittest.skipUnless((WORKSPACE / '.runtime/bin/mcap').exists(), 'MCAP CLI not installed')
    def test_failed_reindex_remains_pending_and_can_be_retried(self):
        directory, path = self.create_bag('reindex_retry')
        damaged = path.read_bytes()[:-37]
        path.write_bytes(damaged)
        real_run = subprocess.run

        def fail_reindex(args, **kwargs):
            if args[:3] == ['ros2', 'bag', 'reindex']:
                raise subprocess.CalledProcessError(2, args)
            return real_run(args, **kwargs)

        with unittest.mock.patch.object(finalizer.subprocess, 'run', side_effect=fail_reindex):
            self.assertRaises(subprocess.CalledProcessError, finalizer.finalize, directory)
        self.assertTrue((directory / '.mcap-reindex-required').exists())
        self.assertTrue(finalizer.is_finalized(path))
        self.assertEqual(path.with_suffix('.mcap.damaged').read_bytes(), damaged)
        (directory / 'metadata.yaml').unlink()
        finalizer.finalize(directory)
        self.assertTrue((directory / 'metadata.yaml').is_file())
        self.assertFalse((directory / '.mcap-reindex-required').exists())


if __name__ == '__main__':
    unittest.main()
