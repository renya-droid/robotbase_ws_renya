"""Nonblocking touch-triggered WAV playback and smoothed mouth envelope."""
import array
import math
import random
import shutil
import sys
import time
import wave
from pathlib import Path
from PyQt5.QtCore import QObject, QProcess, QTimer, pyqtSignal

SPECIAL_CUES = ('cavity', 'licking', 'tooth_drop', 'shy', 'crying', 'radiant', 'birthday')

def audio_envelope(path):
    """Read 20ms RMS windows from PCM16 audio, normalized per utterance."""
    with wave.open(str(path)) as stream:
        if stream.getsampwidth() != 2 or stream.getcomptype() != 'NONE':
            raise ValueError(f'PCM16 WAV required: {path}')
        step = max(1, round(stream.getframerate() * .02))
        levels = []
        while True:
            block = stream.readframes(step)
            if not block:
                break
            samples = array.array('h', block)
            if sys.byteorder != 'little':
                samples.byteswap()
            levels.append(math.sqrt(sum(s*s for s in samples) / len(samples)))
    peak = max(levels, default=0)
    if not levels:
        raise ValueError(f'Empty WAV: {path}')
    return [min(1.0, max(0.0, (v / max(1, peak) - .035) / .7)) for v in levels]


class TouchVoice(QObject):
    active_changed = pyqtSignal(bool)
    mouth_level = pyqtSignal(float)

    def __init__(self, directory, probability=.3, cooldown=4.0, parent=None):
        super().__init__(parent)
        self.probability = probability
        self.cooldown = cooldown
        self.clips = []
        for path in sorted(Path(directory).glob('*.wav')):
            try:
                self.clips.append((path, audio_envelope(path)))
            except (ValueError, wave.Error, OSError) as error:
                print(f'音声を読み込めません: {error}', file=sys.stderr)
        self.player = shutil.which('aplay')
        self.process = QProcess(self)
        self.process.started.connect(self._started)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._failed)
        self.timer = QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._update_mouth)
        self.last_finished = -float('inf')
        self.active = False
        self.level = 0.0
        self.envelope = []

    def try_play(self, cue=None):
        """Named reactions always play; ordinary touches choose a general clip."""
        clips = [(path, envelope) for path, envelope in self.clips
                 if (path.stem == cue if cue else path.stem not in SPECIAL_CUES)]
        if not clips or not self.player or self.probability <= 0:
            return False
        # A falling tooth has priority over the smile's cavity clip.
        if cue == 'tooth_drop' and (self.active or self.process.state() != QProcess.NotRunning):
            self.process.blockSignals(True)
            try:
                self.process.kill()
                stopped = self.process.waitForFinished(1000)
            finally:
                self.process.blockSignals(False)
            if not stopped and self.process.state() != QProcess.NotRunning:
                return False
            self._finished()
        if (self.active or
                self.process.state() != QProcess.NotRunning or
                (cue is None and time.monotonic() - self.last_finished < self.cooldown)):
            return False
        if cue is None and random.random() >= self.probability:
            return False
        path, self.envelope = clips[0] if cue else random.choice(clips)
        self.process.start(self.player, ['-q', str(path)])
        return True

    def _started(self):
        self.started_at = time.monotonic()
        self.active = True
        self.level = 0.0
        self.active_changed.emit(True)
        self.mouth_level.emit(0.0)
        self.timer.start()

    def _update_mouth(self):
        index = int((time.monotonic() - self.started_at) / .02)
        target = self.envelope[index] if index < len(self.envelope) else 0.0
        self.level += (target - self.level) * .4
        self.mouth_level.emit(self.level)

    def _finished(self, *args):
        if args and args[0] != 0:
            detail = bytes(self.process.readAllStandardError()).decode(errors='replace').strip()
            print(f'音声再生に失敗しました: {detail}', file=sys.stderr)
        self.timer.stop()
        was_active = self.active
        self.active = False
        self.last_finished = time.monotonic()
        self.mouth_level.emit(0.0)
        if was_active:
            self.active_changed.emit(False)

    def _failed(self, error):
        print(f'音声再生エラー: {self.process.errorString()}', file=sys.stderr)
        self._finished()

    def close(self):
        self.timer.stop()
        self.process.kill()
        self.process.waitForFinished(1000)
        self._finished()
