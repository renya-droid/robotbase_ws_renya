"""Check reaction routing without playing sound or starting ROS nodes."""
import os
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtGui import QKeyEvent
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES, ASSET_DIRECTORY
from touch_voice import TouchVoice


class VoiceRoutingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = FaceWindow(DEFAULT_FACES, speech_probability=1)
        self.window.voice.try_play = Mock(return_value=True)

    def tearDown(self):
        self.window.close()

    def test_teeth_trigger_cavity_and_stay_visible_during_audio(self):
        for state in ('happy', 'smile', 'laughing'):
            self.window.show_state(state)
            self.window.voice.try_play.assert_called_with('cavity')
            self.window.set_speaking(True)
            self.assertEqual(self.window.current_state, state)
            self.window.set_speaking(False)
            self.window.voice.try_play.reset_mock()
            self.window.show_state(state)
            self.window.voice.try_play.assert_not_called()

    def test_tongue_touch_wins_over_rapid_touch_and_random_audio(self):
        for _ in range(3):
            self.window.handle_touch(.50, .69)
            self.assertEqual(self.window.current_state, 'licking')
        self.assertEqual(self.window.voice.try_play.call_count, 3)
        self.window.voice.try_play.assert_called_with('licking')

    def test_shy_and_crying_keep_expression_and_do_not_repeat_voice(self):
        for state in ('shy', 'crying', 'radiant'):
            self.window.show_state(state)
            self.window.voice.try_play.assert_called_with(state)
            self.window.voice.try_play.reset_mock()
            self.window.set_speaking(True)
            self.assertEqual(self.window.current_state, state)
            self.window.set_speaking(False)
            self.window.voice.try_play.assert_not_called()

    def test_other_touch_requests_random_audio(self):
        self.window.handle_touch(.39, .52)
        self.window.voice.try_play.assert_called_once_with()

    def test_birthday_key_plays_only_its_cue(self):
        self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_7, Qt.NoModifier))
        self.window.voice.try_play.assert_called_once_with('birthday')
        self.assertEqual(self.window.current_state, 'radiant')
        self.assertEqual(self.window.face.bubble_text, 'お誕生日おめでとう！')
        self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_7,
                                          Qt.NoModifier, '7', True, 1))
        self.window.voice.try_play.assert_called_once_with('birthday')
        self.window.set_speaking(True)
        self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_7, Qt.NoModifier))
        self.window.voice.try_play.assert_called_once_with('birthday')

    def test_head_pat_is_happy_and_nose_touch_is_surprised(self):
        self.window.handle_touch(.50, .20)
        self.assertEqual(self.window.current_state, 'shy')
        self.window.voice.try_play.assert_called_once_with('shy')
        self.window.handle_touch(.50, .50)
        self.assertEqual(self.window.current_state, 'astonished')

    def test_tooth_loss_falls_then_surprises_then_cries(self):
        self.window.show_state('laughing')
        self.window.set_speaking(True)
        self.window.handle_touch(.484, .615)
        start = self.window.tooth_reaction_started
        self.assertIsNotNone(start)
        self.window.voice.try_play.reset_mock()
        self.window.update_companion(start+1.3)
        self.assertEqual(self.window.current_state, 'laughing')
        self.window.update_companion(start+1.7)
        self.assertEqual(self.window.current_state, 'astonished')
        self.window.update_companion(start+3.5)
        self.assertEqual(self.window.current_state, 'crying')
        self.assertEqual(self.window.speech_resume_state, 'crying')
        self.window.voice.try_play.assert_not_called()
        self.window.update_companion(start+8.5)
        self.assertEqual(self.window.current_state, 'troubled')
        self.window.update_companion(start+14.1)
        self.assertEqual(self.window.current_state, 'normal')
        self.assertIsNone(self.window.tooth_reaction_started)

    def test_idle_mascot_changes_emotion_quietly_and_returns_on_touch(self):
        origin = self.window.face.last_interaction
        self.window.voice.try_play.reset_mock()
        self.window.update_companion(origin+21.)
        self.assertEqual(self.window.current_state, 'thinking')
        self.window.update_companion(origin+29.)
        self.assertEqual(self.window.current_state, 'radiant')
        self.window.voice.try_play.assert_not_called()
        self.window.face.note_interaction()
        self.window.update_companion()
        self.assertEqual(self.window.current_state, 'normal')
        self.assertIsNone(self.window.idle_origin_state)

    def test_explicit_expression_cancels_tooth_reaction(self):
        self.window.show_state('laughing')
        self.window.handle_touch(.484, .615)
        self.window.show_state('astonished')
        self.assertIsNone(self.window.tooth_reaction_started)

    def test_tooth_falls_once_during_voice_and_resets_on_new_expression(self):
        self.window.show_state('laughing')
        self.window.set_speaking(True)
        self.window.handle_touch(.484, .615)
        self.assertTrue(self.window.face.tooth_missing)
        self.window.voice.try_play.assert_called_with('tooth_drop')
        started = self.window.face.tooth_started
        self.window.handle_touch(.484, .615)
        self.assertEqual(self.window.face.tooth_started, started)
        self.window.set_speaking(False)
        self.assertTrue(self.window.face.tooth_missing)
        self.window.show_state('normal')
        self.assertFalse(self.window.face.tooth_missing)
        self.assertFalse(self.window.face.tooth_timer.isActive())
        self.window.show_state('laughing')
        self.assertFalse(self.window.face.drop_front_tooth(.1, .1))
        self.assertTrue(self.window.face.drop_front_tooth(.484, .615))

    def test_letterboxed_touch_targets_front_tooth(self):
        self.window.resize(800, 800)
        self.window.show()
        self.app.processEvents()
        self.window.show_state('laughing')
        target = self.window.face.face_rect()
        x = (target.x() + .484 * target.width()) / self.window.face.width()
        y = (target.y() + .615 * target.height()) / self.window.face.height()
        self.window.handle_face_touch(x, y)
        self.assertTrue(self.window.face.tooth_missing)

    def test_player_uses_reserved_clips_only_for_matching_cues(self):
        voice = TouchVoice(ASSET_DIRECTORY / 'voice', probability=1)
        voice.player = '/mock/aplay'
        try:
            with patch.object(voice.process, 'start') as start:
                for cue in ('cavity', 'licking', 'tooth_drop', 'shy', 'crying', 'radiant', 'birthday'):
                    voice.last_finished = float('inf')  # special cues bypass cooldown
                    self.assertTrue(voice.try_play(cue))
                    self.assertTrue(start.call_args.args[1][1].endswith(cue + '.wav'))
                voice.last_finished = -float('inf')
                for _ in range(20):
                    self.assertTrue(voice.try_play())
                    name = start.call_args.args[1][1]
                    self.assertTrue(name.endswith(('hello.wav', 'introduction.wav')))
                voice.probability = 0
                self.assertFalse(voice.try_play('cavity'))
        finally:
            voice.close()


if __name__ == '__main__':
    unittest.main()
