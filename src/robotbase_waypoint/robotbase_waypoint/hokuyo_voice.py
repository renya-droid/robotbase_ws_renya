"""Play the two Hokuyo transition cues without blocking waypoint navigation."""

import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


class HokuyoVoice:
    """Play one cue per actual costmap state change."""

    def __init__(self, logger):
        """Locate the installed WAV files and audio player."""
        self.logger = logger
        self.player = shutil.which('aplay')
        self.voice_dir = Path(get_package_share_directory('robotbase_waypoint')) / 'voice'
        self.process = None
        if self.player is None:
            self.logger.warning('Hokuyo voice unavailable: aplay was not found')

    def play(self, disabled: bool) -> None:
        """Start a cue asynchronously, replacing a cue still playing."""
        self.check()
        if self.player is None:
            return
        path = self.voice_dir / ('hokuyo_off.wav' if disabled else 'hokuyo_on.wav')
        if not path.is_file():
            self.logger.warning(f'Hokuyo voice file not found: {path}')
            return
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.stderr.close()
            self.process = None
        try:
            self.process = subprocess.Popen(
                [self.player, '-q', str(path)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, start_new_session=True)
            self.logger.info(f'Hokuyo voice started: {path.name}')
        except OSError as error:
            self.logger.warning(f'Hokuyo voice playback failed: {error}')

    def check(self) -> None:
        """Report playback errors from the child without blocking navigation."""
        if self.process is None:
            return
        result = self.process.poll()
        if result is None:
            return
        if result != 0:
            detail = self.process.stderr.read().decode(errors='replace').strip()
            self.logger.warning(f'Hokuyo voice playback failed ({result}): {detail}')
        self.process.stderr.close()
        self.process = None
