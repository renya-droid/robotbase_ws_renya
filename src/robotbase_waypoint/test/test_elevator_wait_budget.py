from test_elevator_task import running, packet


def publish_wait(f, sequence, phase='WAIT_START'):
    f._elevator_state(packet(state_sequence=sequence, phase=phase, finished=False,
                             waypoint_cycle_result=''))


def test_ten_minute_live_preboarding_wait_does_not_end_cycle(monkeypatch):
    f, now = running(monkeypatch)
    for seq in range(601):
        now[0] = 100. + seq
        publish_wait(f, seq+1)
        f._continue_elevator()
    assert f.elevator_task['stage'] == 'cycle' and not f.finished
    assert f.requests == ['request'] and f.messages == [False] and not f.advances


def test_active_budget_still_expires_after_wait(monkeypatch):
    f, now = running(monkeypatch)
    for seq in range(601):
        now[0] = 100. + seq; publish_wait(f, seq+1, 'WAIT_SKIP_OPEN'); f._continue_elevator()
    for seq in range(1, 303):
        now[0] = 700. + seq
        publish_wait(f, 601+seq, 'ENTERING'); f._continue_elevator()
    assert f.finished and f.elevator_task['stage'] == 'fault' and f.messages[-1] == 'stop'


def test_lost_state_stream_still_fails_during_wait(monkeypatch):
    f, now = running(monkeypatch); publish_wait(f, 1)
    now[0] += 4.1; f._continue_elevator()
    assert f.finished and f.elevator_task['stage'] == 'fault' and not f.advances


def test_duplicate_state_cannot_keep_wait_alive(monkeypatch):
    f, now = running(monkeypatch); publish_wait(f, 1)
    now[0] += 4.1; publish_wait(f, 1); f._continue_elevator()
    assert f.finished and f.elevator_task['stage'] == 'fault'


def test_wait_does_not_erase_earlier_active_time(monkeypatch):
    f, now = running(monkeypatch)
    for seq in range(151):
        now[0] = 100. + seq; publish_wait(f, seq+1, 'ENTERING'); f._continue_elevator()
    for seq in range(601):
        now[0] = 250. + seq; publish_wait(f, seq+152); f._continue_elevator()
    for seq in range(152):
        now[0] = 850. + seq; publish_wait(f, seq+753, 'ENTERING'); f._continue_elevator()
    assert f.finished and f.elevator_task['stage'] == 'fault'
