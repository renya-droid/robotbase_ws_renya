"""Verify splitting, bounded movement, transparent sprites and automatic merging."""
import os
import unittest
from unittest.mock import Mock, patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QEvent, QRectF, Qt
from PyQt5.QtGui import QKeyEvent
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES
from split_play import SplitPlay


class SplitPlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_touch_splits_until_limit_then_scatters(self):
        play = SplitPlay(); play.start(0.)
        self.assertEqual(len(play.faces), 2)
        for i in range(20):
            self.assertTrue(play.touch(.4, .6, now=.1*i))
            self.assertLessEqual(len(play.faces), 8)
        self.assertEqual(len(play.faces), 8)
        self.assertTrue(all(face['target'] >= .23 for face in play.faces))

    def test_emotions_are_distinct_and_change_on_independent_clocks(self):
        play = SplitPlay(); play.start(0.)
        first, second = play.faces
        original_first, original_second = first['emotion'], second['emotion']
        self.assertNotEqual(original_first, original_second)
        self.assertNotEqual(first['phase'], second['phase'])
        play.advance(first['next_emotion']+.01)
        self.assertNotEqual(first['emotion'], original_first)
        self.assertEqual(second['emotion'], original_second)
        self.assertEqual(first['previous_emotion'], original_first)
        for tick in range(12):
            play.touch(.5, .5, now=4.+tick*.1)
        emotions = [face['emotion'] for face in play.faces]
        self.assertEqual(len(emotions), len(set(emotions)))
        play.advance(9.)
        self.assertEqual(len(play.faces), len({face['emotion'] for face in play.faces}))

    def test_emotions_freeze_while_gathering_and_blend_back_during_growth(self):
        play = SplitPlay(); play.start(0.)
        play.merge(1.)
        emotions = [face['emotion'] for face in play.faces]
        play.advance(2.)
        self.assertEqual([face['emotion'] for face in play.faces], emotions)
        self.assertEqual(play.faces[0]['merge_blend'], 0.)
        play.advance(3.7)
        self.assertGreater(play.faces[0]['merge_blend'], .9)
        self.assertEqual([face['emotion'] for face in play.faces], emotions)

    def test_each_face_renders_its_own_emotion_without_audio(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=False)
        try:
            window.show_state('curious')
            window.start_split_play()
            first, second = window.face.split_play.faces
            first['emotion'], second['emotion'] = 'crying', 'radiant'
            now = first['emotion_since']+1.
            frame = window.movie.currentPixmap()
            with patch('robot_face_display.time.monotonic', return_value=now):
                sample = window.face.motion_sample()
                crying = window.face.split_sprite(first, frame, sample).toImage()
                happy = window.face.split_sprite(second, frame, sample).toImage()
            def blue_tears(image):
                return sum(c.blue() > 130 and c.green() > 100 and c.blue()-c.red() > 35
                           for y in range(150, 230) for x in range(210, 430)
                           for c in [image.pixelColor(x, y)])
            self.assertGreater(blue_tears(crying), 100)
            self.assertEqual(blue_tears(happy), 0)
            self.assertEqual(window.current_state, 'curious')
            self.assertEqual(crying.pixelColor(0, 0).alpha(), 0)
            window.voice.try_play.assert_not_called()
        finally:
            window.close()

    def test_moves_and_shrinks_inside_letterboxed_window(self):
        play = SplitPlay(); play.start(0.)
        for step in range(1, 121):
            play.advance(step/30., face_fraction=(1., .65))
            for face in play.faces:
                self.assertGreaterEqual(face['x'], .27*face['scale'])
                self.assertLessEqual(face['x'], 1.-.27*face['scale'])
                self.assertGreaterEqual(face['y'], .49*face['scale']*.65)
                self.assertLessEqual(face['y'], 1.-.49*face['scale']*.65)
        self.assertLess(play.faces[0]['scale'], .53)
        self.assertNotEqual(play.faces[0]['x'], .5)

    def test_idle_merges_at_center_and_restores_original_size(self):
        play = SplitPlay(); play.start(0.)
        play.touch(.3, .4, now=1.)
        play.advance(5.9)
        self.assertIsNone(play.merge_started)
        play.advance(6.)
        self.assertEqual(play.merge_started, 6.)
        play.advance(8.81)
        self.assertTrue(play.active)
        self.assertEqual(len(play.faces), 2)
        play.advance(10.9)
        self.assertIsNone(play.merge_started)
        play.advance(13.79)
        self.assertAlmostEqual(play.faces[0]['x'], .5, places=3)
        self.assertAlmostEqual(play.faces[0]['y'], .5, places=3)
        self.assertAlmostEqual(play.faces[0]['scale'], 1., places=3)
        self.assertTrue(play.advance(13.81))
        self.assertFalse(play.active)

    def test_touch_during_merge_resumes_play_and_restarts_idle_count(self):
        play = SplitPlay(); play.start(0.)
        play.advance(6.)
        self.assertIsNotNone(play.merge_started)
        play.touch(.6, .2, now=6.1)
        self.assertIsNone(play.merge_started)
        play.advance(10.)
        self.assertTrue(play.active)
        self.assertIsNone(play.merge_started)
        play.advance(18.91)
        self.assertFalse(play.active)

    def test_x_touch_manual_merge_and_auto_repeat_in_window(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=False)
        try:
            window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_X, Qt.NoModifier))
            self.assertTrue(window.face.split_play.active)
            window.handle_face_touch(.3, .4)
            self.assertEqual(len(window.face.split_play.faces), 3)
            window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_X, Qt.NoModifier, 'x', True, 1))
            self.assertIsNone(window.face.split_play.merge_started)
            window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_X, Qt.NoModifier))
            self.assertIsNotNone(window.face.split_play.merge_started)
            started = window.face.split_play.merge_started
            window.face.advance_motion(started+2.81)
            self.assertTrue(window.face.split_play.active)
            self.assertEqual(len(window.face.split_play.faces), 2)
            end = started+5.+2.81
            window.face.advance_motion(end)
            self.assertFalse(window.face.split_play.active)
            self.assertEqual(window.face.last_interaction, end)
            window.voice.try_play.assert_not_called()
        finally:
            window.close()

    def test_only_one_face_merges_each_five_seconds_when_apart(self):
        play = SplitPlay(); play.start(0.)
        for _ in range(6):
            play.touch(.5, .5, now=0.)
        self.assertEqual(len(play.faces), 8)
        for index, face in enumerate(play.faces):
            face.update(x=(index+1)/9., y=.5, scale=.1, target=.1, vx=0., vy=0.)
        play.contacts = play._contacts()
        for turn in range(1, 8):
            play.advance(turn*5.-.01)
            self.assertEqual(len(play.faces), 9-turn)
            play.advance(turn*5.+2.81)
            if turn < 7:
                self.assertTrue(play.active)
                self.assertEqual(len(play.faces), 8-turn)
            else:
                self.assertFalse(play.active)

    def test_other_faces_keep_playing_during_pair_merge(self):
        play = SplitPlay(); play.start(0.)
        play.touch(.5, .5, now=0.)
        play.touch(.5, .5, now=0.)
        play.advance(5.)
        self.assertEqual(len(play.merging_ids), 2)
        other = next(face for face in play.faces if face['id'] not in play.merging_ids)
        before = other['x'], other['y']
        play.advance(5.5)
        self.assertNotEqual((other['x'], other['y']), before)
        self.assertEqual(other['opacity'], 1.)
        self.assertEqual(other['merge_blend'], 0.)

    def test_collision_merges_the_touching_pair_without_waiting_five_seconds(self):
        play = SplitPlay(); play.start(0.)
        play.touch(.5, .5, now=0.)
        for face, position in zip(play.faces, ((.1, .2), (.6, .6), (.9, .6))):
            face.update(x=position[0], y=position[1], scale=.2, target=.2, vx=0., vy=0.)
        play.advance(1.1)
        self.assertIsNone(play.merge_started)
        first, second = play.faces[1:]
        second['x'] = .69
        play.advance(1.2)
        self.assertEqual(set(play.merging_ids), {first['id'], second['id']})
        self.assertEqual(play.merge_started, 1.2)
        play.advance(4.01)
        self.assertTrue(play.active)
        self.assertEqual(len(play.faces), 2)

    def test_overlapping_newborn_faces_do_not_immediately_merge_back(self):
        play = SplitPlay(); play.start(0.)
        for face in play.faces:
            face.update(vx=0., vy=0.)
        play.advance(.2)
        play.advance(1.5)
        self.assertIsNone(play.merge_started)
        self.assertEqual(len(play.faces), 2)
        play.touch(.5, .5, now=1.6)
        play.advance(1.7)
        self.assertIsNone(play.merge_started)
        self.assertEqual(len(play.faces), 3)

    def test_collision_uses_head_shape_and_letterboxing(self):
        play = SplitPlay(); play.start(0.)
        first, second = play.faces
        first.update(x=.5, y=.4, scale=.2)
        second.update(x=.5, y=.55, scale=.2)
        play.face_fraction = (1., 1.)
        self.assertTrue(play._contacts())
        play.face_fraction = (1., .5)
        self.assertFalse(play._contacts())
        second.update(x=.65, y=.4)
        self.assertFalse(play._contacts(), 'Transparent sprite margins must not count as contact')

    def test_splitting_and_butterfly_are_exclusive_and_motion_exits_play(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=False)
        try:
            window.start_butterfly()
            self.assertTrue(window.start_split_play())
            self.assertFalse(window.face.butterfly.active)
            self.assertFalse(window.start_butterfly())
            window.motion_received(True)
            self.assertFalse(window.face.split_play.active)
            self.assertEqual(window.current_state, 'moving')
        finally:
            window.close()

    def test_sprite_background_does_not_cover_other_faces(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=False)
        window.resize(960, 540); window.show(); self.app.processEvents()
        try:
            window.start_split_play()
            play = window.face.split_play
            play.faces[0].update(x=.35, y=.5, scale=.5, angle=0.)
            play.faces[1].update(x=.65, y=.5, scale=.5, angle=0.)
            image = window.face.grab().toImage()
            # Both noses must remain visible: the second sprite's black padding
            # used to obscure the first sprite when transparency was inverted.
            for cx in (.35, .65):
                warm = 0
                for y in range(250, 285):
                    for x in range(round(cx*960)-22, round(cx*960)+22):
                        c = image.pixelColor(x, y)
                        warm += c.red() > 150 and 80 < c.green() < 220 and c.blue() < 190
                self.assertGreater(warm, 30, cx)
        finally:
            window.close()


if __name__ == '__main__':
    unittest.main()
