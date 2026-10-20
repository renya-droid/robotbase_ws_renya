"""Regress the race: dependent nodes must never race Gazebo's preflight."""
import time
from types import SimpleNamespace
from PySide6.QtWidgets import QApplication, QMessageBox
from robot_launcher import RobotLauncher


def make_window(monkeypatch):
    app = QApplication.instance() or QApplication([])
    w = RobotLauncher()
    events = []
    names = ['koko_sim_room', 'koko_nav2_sim_people', 'koko_rviz_sim']
    w.button_map = {name: SimpleNamespace(
        process_manager=SimpleNamespace(is_running=lambda: False),
        launch=lambda name=name: events.append(name)) for name in names}
    monkeypatch.setattr(w, 'wait_for_simulation', lambda: events.append('wait'))
    monkeypatch.setattr(QMessageBox, 'warning', lambda *args: events.append('warning'))
    return app, w, events, names


class Probe:
    def __init__(self):
        self.killed = False
    def readAllStandardOutput(self):
        return b'ready'
    def deleteLater(self):
        pass
    def kill(self):
        self.killed = True


def test_dependents_wait_for_clock_and_odom(monkeypatch):
    app, w, events, names = make_window(monkeypatch)
    w.launch_preset('room', names)
    assert events == ['koko_sim_room', 'wait']
    for _ in range(5):
        app.processEvents()
    assert events == ['koko_sim_room', 'wait']
    probe = Probe()
    w.preset_probe = probe
    w.simulation_ready(probe, 0)
    until = time.monotonic()+1
    while len(events)<4 and time.monotonic()<until:
        app.processEvents(); time.sleep(.005)
    assert events == ['koko_sim_room', 'wait', 'koko_nav2_sim_people', 'koko_rviz_sim']
    w.close()


def test_failed_gazebo_aborts_preset(monkeypatch):
    app, w, events, names = make_window(monkeypatch)
    w.launch_preset('room', names)
    probe = Probe(); w.preset_probe = probe
    w.simulation_ready(probe, 1)
    app.processEvents()
    assert events == ['koko_sim_room', 'wait', 'warning']
    assert not w.preset_queue
    w.close()


def test_stop_cancels_pending_start_and_stale_success(monkeypatch):
    app, w, events, names = make_window(monkeypatch)
    w.launch_preset('room', names)
    probe = Probe(); w.preset_probe = probe
    w.cancel_preset()
    w.simulation_ready(probe, 0)
    app.processEvents()
    assert probe.killed and not w.preset_queue
    assert events == ['koko_sim_room', 'wait']
    w.close()
