"""Exercise complete games, interruption and actual pointer event routing."""
import os
import unittest
from unittest.mock import Mock, patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QEvent, QPoint, QPointF, Qt
from PyQt5.QtGui import QMouseEvent
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES
from companion_play import CompanionPlay
from split_play import SplitPlay


class CompanionPlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = FaceWindow(DEFAULT_FACES, speech_probability=0, transparent_background=True)
        self.window.voice.try_play = Mock(return_value=False)
        self.window.resize(960, 540)
        self.window.show(); self.app.processEvents()
        self.window.pointer_timer.stop(); self.window.companion_timer.stop()
        self.window.face.animation_timer.stop()

    def tearDown(self):
        self.window.close()

    def test_tickle_laughs_then_pouts_and_expires(self):
        with patch('robot_face_display.time.monotonic', return_value=100.):
            for _ in range(3):
                self.window.handle_touch(.39, .52)
            self.assertEqual(self.window.current_state, 'laughing')
            for _ in range(3):
                self.window.handle_touch(.61, .52)
            self.assertEqual(self.window.current_state, 'pout')
        with patch('robot_face_display.time.monotonic', return_value=104.):
            self.window.handle_touch(.39, .52)
            self.assertEqual(self.window.current_state, 'surprise')

    def test_drag_releases_with_velocity_and_never_becomes_a_tap(self):
        face = self.window.face
        tap = Mock(); face.touched.connect(tap)
        def event(kind, x, y, button, buttons):
            return QMouseEvent(kind, QPointF(x, y), button, buttons, Qt.NoModifier)
        with patch('robot_face_display.time.monotonic', return_value=100.):
            face.mousePressEvent(event(QEvent.MouseButtonPress, 480, 270, Qt.LeftButton, Qt.LeftButton))
        with patch('robot_face_display.time.monotonic', return_value=100.1):
            face.mouseMoveEvent(event(QEvent.MouseMove, 570, 290, Qt.NoButton, Qt.LeftButton))
            self.assertEqual(face.play.mode, 'bounce')
            self.assertTrue(face.play.held)
            face.mouseReleaseEvent(event(QEvent.MouseButtonRelease, 570, 290, Qt.LeftButton, Qt.NoButton))
        self.assertFalse(face.play.held)
        self.assertGreater(face.play.vx, 0.)
        tap.assert_not_called()
        for step in range(1, 120):
            face.play.advance(100.1+step*.05)
            self.assertTrue(.16 <= face.play.x <= .84)
            self.assertTrue(.28 <= face.play.y <= .72)
        self.window.update_companion(106.2)
        self.assertIsNone(face.play.mode)
        self.assertEqual(self.window.current_state, 'normal')

    def test_voice_does_not_block_tickle_reaction_or_play_more_clips(self):
        self.window.set_speaking(True)
        with patch('robot_face_display.time.monotonic', return_value=100.):
            for _ in range(6):
                self.window.handle_touch(.39, .52)
        self.assertEqual(self.window.current_state, 'pout')
        self.assertEqual(self.window.speech_resume_state, 'pout')
        self.window.voice.try_play.assert_not_called()

    def test_food_pointer_drag_is_consumed_instead_of_dragging_face(self):
        w = self.window; w.start_game('food')
        face = w.face
        food = QPointF(face.width()*.79, face.height()*.72)
        target = face.face_rect()
        mouth = face.motion.transform(target, face.motion_sample()).map(
            QPointF(target.width()*.5+target.x(), target.height()*.68+target.y()))
        face.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, food, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
        self.assertTrue(face.play.food_held)
        face.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, mouth, Qt.NoButton, Qt.LeftButton, Qt.NoModifier))
        face.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, mouth, Qt.LeftButton, Qt.NoButton, Qt.NoModifier))
        self.assertEqual(face.play.mode, 'eating')
        self.assertFalse(face.play.food_held)

    def test_feed_misses_reset_and_mouth_drop_chews_then_satisfies(self):
        w = self.window
        w.start_game('food')
        w.face.play.food = (.1, .1)
        w.finish_food()
        self.assertEqual(w.face.play.mode, 'food')
        self.assertEqual(w.face.play.food, (.79, .72))
        target, sample = w.face.face_rect(), w.face.motion_sample()
        mouth = w.face.motion.transform(target, sample).map(
            QPointF(target.x()+target.width()*.5, target.y()+target.height()*.68))
        w.face.play.food = (mouth.x()/w.face.width(), mouth.y()/w.face.height())
        w.finish_food()
        self.assertEqual(w.face.play.mode, 'eating')
        started = w.face.play.started
        with patch('robot_face_display.time.monotonic', return_value=started+.5):
            first = w.face.motion_sample()['mouth_open']
        with patch('robot_face_display.time.monotonic', return_value=started+.6):
            second = w.face.motion_sample()['mouth_open']
        self.assertNotAlmostEqual(first, second)
        w.update_companion(started+3.1)
        self.assertEqual(w.current_state, 'relieved')
        w.update_companion(started+4.6)
        self.assertIsNone(w.face.play.mode)
        self.assertEqual(w.current_state, 'normal')

    def test_sleep_touch_startles_and_restores_previous_expression(self):
        w = self.window
        w.show_state('curious'); w.start_game('sleep', now=10.)
        w.update_companion(13.1)
        self.assertEqual(w.current_state, 'sleep')
        with patch('robot_face_display.time.monotonic', return_value=15.):
            w.handle_face_touch(.5, .5)
        self.assertEqual(w.current_state, 'astonished')
        self.assertEqual(w.face.play.mode, 'wake')
        w.update_companion(16.6)
        self.assertEqual(w.current_state, 'curious')

    def test_idle_reaches_butterfly_hide_then_sleep_without_looping(self):
        w = self.window
        origin = w.face.last_interaction
        w.update_companion(origin+39.)
        self.assertTrue(w.face.butterfly.active)
        w.update_companion(origin+58.)
        w.update_companion(origin+60.1)
        self.assertEqual(w.face.play.mode, 'hide')
        w.update_companion(origin+69.2)
        self.assertIsNone(w.face.play.mode)
        w.update_companion(origin+80.1)
        self.assertEqual(w.face.play.mode, 'sleep')
        w.update_companion(origin+84.)
        self.assertEqual(w.current_state, 'sleep')

    def test_hide_is_partially_off_screen_and_returns_to_original_pose(self):
        w = self.window
        w.start_game('hide', now=10.)
        with patch('robot_face_display.time.monotonic', return_value=14.):
            scene = w.face.render_scene()
            mask = w.face.desktop_input_region(scene)
            self.assertTrue(mask.contains(QPoint(int(w.face.width()*.98), w.face.height()//2)))
            sample = w.face.motion_sample()
            self.assertAlmostEqual(sample['scale'], .42)
            centre = w.face.motion.transform(w.face.face_rect(), sample).map(QPointF(w.face.face_rect().center()))
            self.assertGreater(centre.x(), w.face.width()*.95)
        w.update_companion(19.1)
        self.assertIsNone(w.face.play.mode)

    def test_motion_and_expression_changes_cancel_all_games(self):
        for mode in ('food', 'sleep', 'hide', 'bounce'):
            self.window.show_state('normal')
            self.window.start_game(mode)
            self.window.motion_received(True)
            self.assertIsNone(self.window.face.play.mode)
            self.assertEqual(self.window.current_state, 'moving')
        self.window.show_state('normal'); self.window.start_game('food')
        self.window.start_split_play()
        self.assertIsNone(self.window.face.play.mode)
        self.assertTrue(self.window.face.split_play.active)

    def test_social_reactions_and_drag_keep_merge_contract(self):
        play = SplitPlay(); play.start(0.)
        first, second = play.faces
        first.update(x=.3, y=.5, scale=.2, target=.2, emotion='crying')
        second.update(x=.65, y=.5, scale=.2, target=.2, emotion='curious')
        play.advance(1.9)
        self.assertEqual(second['social_text'], 'だいじょうぶ？')
        self.assertEqual(second['social_emotion'], 'shy')
        self.assertLess(second['vx'], 0.)
        first['emotion'] = 'radiant'
        play._social_reaction(4.6)
        self.assertEqual(second['social_emotion'], 'laughing')
        play.merge(4.7)
        play.drag_to(first['x'], first['y'], .4, .4, 4.8)
        self.assertIsNone(play.merge_started)
        play.advance(20.)
        self.assertTrue(play.active)
        self.assertIsNone(play.merge_started)
        play.release_drag(20.)
        play.advance(24.9)
        self.assertIsNone(play.merge_started)
        play.advance(25.)
        self.assertIsNotNone(play.merge_started)


if __name__ == '__main__':
    unittest.main()
