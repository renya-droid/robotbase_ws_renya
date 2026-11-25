"""Exercise the butterfly lifecycle and the new native expressions offscreen."""
import os
import unittest
from unittest.mock import Mock, patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QEvent, QPointF, QRectF, Qt
from PyQt5.QtGui import QKeyEvent, QMovie, QPainter, QPixmap
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES, DEFAULT_STATES
from expression_art import EXTRA_EXPRESSIONS
from butterfly import Butterfly


class ButterflyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        self.window.voice.try_play = Mock(return_value=False)
        self.window.pointer_timer.stop()
        self.window.companion_timer.stop()

    def tearDown(self):
        self.window.close()

    def test_keys_select_distinct_expressions_despite_shared_base_asset(self):
        for key, state in zip((Qt.Key_5, Qt.Key_6, Qt.Key_0, Qt.Key_8, Qt.Key_9), EXTRA_EXPRESSIONS):
            self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier))
            self.assertEqual(self.window.current_state, state)
            self.assertEqual(self.window.face.motion_sample()['expression'], state)

    def test_ros_state_name_starts_visit_and_b_key_recalls_it(self):
        self.assertTrue(self.window.show_state('butterfly'))
        self.assertTrue(self.window.face.butterfly.active)
        self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_B, Qt.NoModifier))
        self.assertTrue(self.window.face.butterfly.active)
        self.assertEqual(self.window.current_state, 'curious')

    def test_b_can_restart_repeatedly_during_and_after_a_visit(self):
        origin = self.window.face.last_interaction
        def press_at(now, autorepeat=False):
            with patch('robot_face_display.time.monotonic', return_value=now):
                self.window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_B,
                                                    Qt.NoModifier, 'b', autorepeat, 1))
        press_at(origin+1.)
        first = self.window.face.butterfly.started
        self.window.update_companion(first+4.)
        self.assertEqual(self.window.current_state, 'excited')
        press_at(first+5.)
        self.assertEqual(self.window.face.butterfly.started, first+5.)
        self.assertEqual(self.window.current_state, 'curious')
        press_at(first+5.1, autorepeat=True)
        self.assertEqual(self.window.face.butterfly.started, first+5.)
        self.window.update_companion(first+23.1)
        self.assertFalse(self.window.face.butterfly.active)
        press_at(first+23.2)
        self.assertTrue(self.window.face.butterfly.active)
        self.assertEqual(self.window.face.butterfly.started, first+23.2)
        self.assertEqual(self.window.butterfly_resume_state, 'normal')
        self.assertIsNone(self.window.face.return_sample)
        self.window.voice.try_play.assert_not_called()

    def test_new_brows_have_solid_hair_and_no_white_eye_fill(self):
        movie = QMovie(str(DEFAULT_STATES['normal'])); movie.jumpToFrame(0)
        source = movie.currentPixmap()
        for state in EXTRA_EXPRESSIONS:
            sample = self.window.face.motion.sample(state, elapsed=1.)
            sample.update(sway=0., bob=0., tilt=0., scale=1.)
            canvas = QPixmap(source.size()); canvas.fill(Qt.black)
            p = QPainter(canvas)
            self.window.face.motion.draw(p, QRectF(canvas.rect()), source, sample, state)
            p.end(); image = canvas.toImage()
            for left, right in ((.355, .479), (.521, .645)):
                hair, white = 0, 0
                for y in range(round(source.height()*.20), round(source.height()*.32)):
                    for x in range(round(source.width()*left), round(source.width()*right)):
                        c = image.pixelColor(x, y)
                        hair += 35 < c.red() < 120 and 30 < c.green() < 110 and 25 < c.blue() < 105
                        white += c.red() > 180 and c.green() > 160 and c.blue() > 130
                self.assertGreater(hair, 400, (state, left))
                self.assertEqual(white, 0, (state, left))

    def test_visit_notices_chases_waves_and_returns_without_audio(self):
        self.window.show_state('pout')
        start = self.window.face.last_interaction+1.
        self.assertTrue(self.window.start_butterfly(start))
        self.assertEqual(self.window.current_state, 'curious')
        self.window.update_companion(start+3.)
        self.assertEqual(self.window.current_state, 'excited')
        self.window.update_companion(start+15.)
        self.assertEqual(self.window.current_state, 'relieved')
        self.window.update_companion(start+18.1)
        self.assertEqual(self.window.current_state, 'pout')
        self.assertFalse(self.window.face.butterfly.active)
        self.window.voice.try_play.assert_not_called()

    def test_gaze_tracks_butterfly_and_chase_stays_inside_screen(self):
        self.window.resize(960, 540)
        self.window.show(); self.app.processEvents()
        start = self.window.face.last_interaction
        self.window.start_butterfly(start)
        target = self.window.face.face_rect()
        with patch('robot_face_display.time.monotonic', return_value=start+4.):
            self.window.update_pointer_gaze()
            point = Butterfly.position(4.)
            self.assertAlmostEqual(self.window.face.gaze_x, (point.x()-.5)*2.*.24)
        for t in (0., 1., 3., 7., 12., 17.):
            with patch('robot_face_display.time.monotonic', return_value=start+t):
                sample = self.window.face.motion_sample()
                transform = self.window.face.motion.transform(target, sample)
                for x, y in ((.25, .42), (.75, .42), (.5, .065), (.5, .965)):
                    self.assertTrue(self.window.face.rect().contains(transform.map(
                        QPointF(target.x()+target.width()*x, target.y()+target.height()*y)).toPoint()))

    def test_user_state_and_robot_motion_cancel_visit(self):
        self.window.start_butterfly()
        self.window.show_state('crying')
        self.assertFalse(self.window.face.butterfly.active)
        self.assertEqual(self.window.current_state, 'crying')
        self.window.start_butterfly()
        self.window.motion_received(True)
        self.assertFalse(self.window.face.butterfly.active)
        self.assertEqual(self.window.current_state, 'moving')
        self.assertFalse(self.window.start_butterfly())

    def test_visit_does_not_interrupt_speech_or_tooth_reaction(self):
        self.window.set_speaking(True)
        self.assertFalse(self.window.start_butterfly())
        self.window.set_speaking(False)
        self.window.tooth_reaction_started = 1.
        self.assertFalse(self.window.start_butterfly())

    def test_idle_visit_waits_then_does_not_restart_immediately(self):
        origin = self.window.face.last_interaction
        self.window.update_companion(origin+37.)
        self.assertFalse(self.window.face.butterfly.active)
        self.window.update_companion(origin+39.)
        self.assertTrue(self.window.face.butterfly.active)
        self.window.update_companion(origin+58.)
        self.window.update_companion(origin+59.)
        self.assertFalse(self.window.face.butterfly.active)

    def test_expression_drawing_preserves_nose(self):
        source_movie = QMovie(str(DEFAULT_STATES['normal'])); source_movie.jumpToFrame(0)
        source = source_movie.currentPixmap()
        original = source.toImage()
        for state in EXTRA_EXPRESSIONS:
            sample = self.window.face.motion.sample(state, elapsed=1.)
            sample.update(sway=0., bob=0., tilt=0., scale=1.)
            canvas = QPixmap(source.size()); canvas.fill(Qt.black)
            p = QPainter(canvas)
            self.window.face.motion.draw(p, QRectF(canvas.rect()), source, sample, state)
            p.end(); image = canvas.toImage()
            count = 0
            for y in range(round(source.height()*.40), round(source.height()*.55)):
                for x in range(round(source.width()*.47), round(source.width()*.53)):
                    color = original.pixelColor(x, y)
                    if color.red() > 150 and 90 < color.green() < 210:
                        self.assertEqual(color, image.pixelColor(x, y), (state, x, y))
                        count += 1
            self.assertGreater(count, 100)

    def test_landing_sneeze_and_recall_are_synchronized(self):
        start = self.window.face.last_interaction
        self.window.start_butterfly(start)
        for t, state in ((9.5, 'curious'), (10.7, 'sneezing'), (11.2, 'sneezing'), (12.4, 'curious')):
            with patch('robot_face_display.time.monotonic', return_value=start+t):
                self.window.update_companion(start+t)
                self.assertEqual(self.window.current_state, state)
                sample = self.window.face.motion_sample()
                self.assertEqual(sample['expression'], state)
                if t == 9.5:
                    self.assertAlmostEqual(sample['cross_eye'], 1.)
                if t == 11.2:
                    self.assertLess(sample['eye_factor'], .15)
                    self.assertGreater(sample['mouth_open'], .9)
                    self.assertIn('はっくしょん', self.window.face.bubble_text)
        with patch('robot_face_display.time.monotonic', return_value=start+12.5):
            self.assertTrue(self.window.recall_butterfly())
            self.assertEqual(self.window.face.motion_sample()['cross_eye'], 0.)
            self.assertEqual(self.window.current_state, 'curious')
        self.window.voice.try_play.assert_not_called()

    def test_butterfly_feet_follow_nose_and_escape_continuously(self):
        for canvas in (QRectF(0., 0., 960., 540.), QRectF(40., 130., 600., 400.)):
            nose = QPointF(canvas.center().x(), canvas.center().y())
            size = Butterfly.size(canvas)
            perch = Butterfly.screen_position(canvas, 9.5, nose)
            self.assertAlmostEqual(perch.x(), nose.x())
            self.assertAlmostEqual(perch.y()+.46*size, nose.y())
            shifted = Butterfly.screen_position(canvas, 9.5, nose+QPointF(16., 7.))
            self.assertAlmostEqual(shifted.x()-perch.x(), 16.)
            self.assertAlmostEqual(shifted.y()-perch.y(), 7.)
            before = Butterfly.screen_position(canvas, Butterfly.SNEEZE-.001, nose)
            after = Butterfly.screen_position(canvas, Butterfly.SNEEZE+.001, nose)
            self.assertLess((after-before).manhattanLength(), 1.)
            away = Butterfly.screen_position(canvas, Butterfly.SNEEZE+1., nose)
            self.assertGreater(away.x()-perch.x(), canvas.width()*.5)
            self.assertLess(away.y(), perch.y()-canvas.height()*.3)

    def test_sneeze_puff_is_visible_only_during_burst(self):
        butterfly = Butterfly()
        canvas_rect = QRectF(0., 0., 960., 540.)
        for elapsed, visible in ((11., False), (11.35, True), (12., False)):
            canvas = QPixmap(960, 540); canvas.fill(Qt.black)
            p = QPainter(canvas)
            butterfly.paint_sneeze(p, canvas_rect, QPointF(480., 300.), elapsed)
            p.end(); image = canvas.toImage()
            white = sum(image.pixelColor(x, y).red() > 50
                        for x in range(440, 520) for y in range(290, 360))
            self.assertEqual(white > 10, visible)

    def test_wings_render_visible_and_path_enters_and_leaves(self):
        butterfly = Butterfly(); butterfly.start(10.)
        self.assertLess(butterfly.position(0.).x(), 0.)
        self.assertGreater(butterfly.position(18.).x(), 1.)
        canvas = QPixmap(960, 540); canvas.fill(Qt.black)
        p = QPainter(canvas); butterfly.paint(p, QRectF(canvas.rect()), now=14.); p.end()
        image = canvas.toImage()
        colored = sum(image.pixelColor(x, y).red() > 80
                      for x in range(0, 960, 3) for y in range(0, 540, 3))
        self.assertGreater(colored, 20)


if __name__ == '__main__':
    unittest.main()
