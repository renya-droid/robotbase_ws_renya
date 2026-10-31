import pytest
from restaurant_sim.motion_monitor import MotionMonitor


def agent():
    return dict(name='customer_00', active=True, done=False, wait=0,
                index=0, steps=[('exit', [5.8, 12.1], 0)], pos=[5.8, 11.0])


def test_stall_fails_at_twenty_seconds_with_context():
    monitor = MotionMonitor()
    a = agent()
    b = dict(agent(), name='customer_01', pos=[5.8, 11.6])
    for second in range(20):
        monitor.observe(a, second, [a, b], 'No route')
    with pytest.raises(RuntimeError, match='MOTION_STALL') as failure:
        monitor.observe(a, 20, [a, b], 'No route')
    assert 'customer_01' in str(failure.value)
    assert 'No route' in str(failure.value)
    assert '"activity": "exit"' in str(failure.value)


def test_tiny_movements_do_not_reset_stall_window():
    monitor = MotionMonitor()
    a = agent()
    with pytest.raises(RuntimeError, match='MOTION_STALL'):
        for second in range(21):
            a['pos'][0] += .001
            monitor.observe(a, second, [a])


def test_intentional_wait_is_excluded_then_motion_is_monitored():
    monitor = MotionMonitor()
    a = agent()
    for second in range(150):
        a['wait'] = 1
        monitor.observe(a, second, [a])
    a['wait'] = 0
    for second in range(150, 170):
        monitor.observe(a, second, [a])
    with pytest.raises(RuntimeError, match='MOTION_STALL'):
        monitor.observe(a, 170, [a])


def test_real_motion_passes_but_subsequent_stop_fails():
    monitor = MotionMonitor()
    a = agent()
    for second in range(40):
        a['pos'][0] += .02
        monitor.observe(a, second, [a])
    with pytest.raises(RuntimeError, match='MOTION_STALL'):
        for second in range(40, 61):
            monitor.observe(a, second, [a])
