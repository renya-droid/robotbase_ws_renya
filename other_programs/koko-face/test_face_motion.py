"""Verify continuous animation geometry and the moving tooth's touch mapping."""
import os
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QPointF
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES, DEFAULT_STATES
from face_motion import FaceMotion, interpolate


class MotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_warp_is_invertible_for_all_expressions_and_audio_levels(self):
        motion = FaceMotion()
        for state in DEFAULT_STATES:
            for t in (0., 1.3, 3.9, 4.15, 4.3, 7.7):
                for level in (0., .5, 1.):
                    points = motion.sample(state, speaking=True, mouth_level=level,
                                           elapsed=t)['points']
                    self.assertTrue(all(x0 < x1 and y0 < y1 for (x0, y0), (x1, y1)
                                        in zip(points, points[1:])), (state, t, level))

    def test_moving_tooth_can_be_touched_at_its_displayed_position(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=True)
        window.resize(900, 700)
        window.show()
        self.app.processEvents()
        window.show_state('laughing')
        try:
            sample = window.face.motion_sample(elapsed=4.15)
            window.face.last_motion_sample = sample
            target = window.face.face_rect()
            y = interpolate(.615, sample['points'])
            local = QPointF(target.x() + .484 * target.width(), target.y() + y * target.height())
            displayed = FaceMotion.transform(target, sample).map(local)
            window.handle_face_touch(displayed.x()/window.face.width(),
                                     displayed.y()/window.face.height())
            self.assertTrue(window.face.tooth_missing)
            window.voice.try_play.assert_called_with('tooth_drop')
        finally:
            window.close()
        self.assertFalse(window.face.animation_timer.isActive())

    def test_static_face_animates_and_audio_deforms_its_mouth(self):
        motion = FaceMotion()
        for state in ('crying', 'shy', 'radiant', 'astonished'):
            a = motion.sample(state, elapsed=1.)
            b = motion.sample(state, elapsed=2.)
            self.assertNotEqual(a['points'], b['points'])
            silent = motion.sample(state, speaking=True, mouth_level=0., elapsed=1.)
            loud = motion.sample(state, speaking=True, mouth_level=1., elapsed=1.)
            self.assertNotEqual(silent['points'], loud['points'])

    def test_speech_keeps_every_current_expression(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        try:
            for state in DEFAULT_STATES:
                window.show_state(state)
                window.set_speaking(True)
                window.set_mouth_level(.8)
                self.assertEqual(window.current_state, state)
                self.assertTrue(window.face.talk_active)
                self.assertEqual(window.face.talk_level, .8)
                window.set_speaking(False)
                self.assertFalse(window.face.talk_active)
        finally:
            window.close()

    def test_silence_and_rest_can_close_lips(self):
        motion = FaceMotion()
        for state in DEFAULT_STATES:
            sample = motion.sample(state, speaking=True, mouth_level=0., elapsed=2.)
            self.assertLess(sample['mouth_open'], .08)
        for state in ('laughing', 'radiant', 'crying', 'astonished'):
            sample = motion.sample(state, elapsed=5.1)
            self.assertLess(sample['mouth_open'], .08)

    def test_idle_movement_waits_then_keeps_contour_inside_screen(self):
        from PyQt5.QtCore import QRectF
        motion = FaceMotion()
        target = QRectF(0., 0., 960., 540.)
        self.assertEqual(motion.sample('normal', idle_seconds=12.)['idle_blend'], 0.)
        positions = []
        for seconds in (17., 25., 40., 60., 95., 150.):
            sample = motion.sample('normal', idle_seconds=seconds, elapsed=seconds)
            transform = FaceMotion.transform(target, sample)
            positions.append(transform.map(target.center()))
            for x, y in ((.25, .42), (.75, .42), (.5, .065), (.5, .965)):
                self.assertTrue(target.contains(transform.map(QPointF(x*960., y*540.))))
        self.assertGreater(max(p.x() for p in positions)-min(p.x() for p in positions), 100.)

    def test_interaction_eases_idle_face_back_without_jump(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        try:
            sample = window.face.motion.sample('normal', elapsed=30., idle_seconds=30.)
            window.face.last_motion_sample = sample
            window.face.note_interaction()
            returning = window.face.motion_sample()
            self.assertAlmostEqual(returning['sway'], sample['sway'], places=3)
            self.assertAlmostEqual(returning['scale'], sample['scale'], places=3)
        finally:
            window.close()

    def test_moustache_pixels_stay_fixed_while_mouth_opens_and_closes(self):
        from PyQt5.QtCore import QRectF
        from PyQt5.QtGui import QPainter, QPixmap
        for state in ('radiant', 'laughing', 'shy', 'crying'):
            source = QPixmap(str(DEFAULT_STATES[state]))
            original = source.toImage()
            results = []
            motion = FaceMotion()
            for level in (0., 1.):
                canvas = QPixmap(source.size())
                canvas.fill()
                painter = QPainter(canvas)
                sample = motion.sample(state, speaking=True, mouth_level=level, elapsed=2.)
                sample.update(sway=0., bob=0., tilt=0., scale=1., eye_shift=0.)
                try:
                    motion.draw(painter, QRectF(canvas.rect()), source, sample, state)
                finally:
                    painter.end()
                results.append(canvas.toImage())
            checked = 0
            for y in range(round(source.height()*.55), round(source.height()*.65), 3):
                for x in range(round(source.width()*.40), round(source.width()*.60), 3):
                    color = original.pixelColor(x, y)
                    if (25 <= max(color.red(), color.green(), color.blue()) <= 130
                            and max(color.red(), color.green(), color.blue())
                            - min(color.red(), color.green(), color.blue()) < 25):
                        # Sample solid hair, not isolated gray compression
                        # speckles inside the original mouth that must be removed.
                        neighbors = [original.pixelColor(x+dx, y+dy)
                                     for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
                        if sum(25 <= max(c.red(), c.green(), c.blue()) <= 130
                               and max(c.red(), c.green(), c.blue())-min(c.red(), c.green(), c.blue()) < 25
                               for c in neighbors) < 7:
                            continue
                        self.assertEqual(results[0].pixelColor(x, y), results[1].pixelColor(x, y),
                                         (state, x, y))
                        checked += 1
            self.assertGreater(checked, 20)

    def test_falling_tooth_is_visible_below_the_mouth_in_widget_render(self):
        import time
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        window.voice.try_play = Mock(return_value=True)
        window.resize(960, 540)
        window.show()
        self.app.processEvents()
        window.pointer_timer.stop()
        window.companion_timer.stop()
        try:
            window.show_state('laughing')
            window.face.previous_pixmap = None
            sample = window.face.motion.sample('laughing', speaking=True, mouth_level=1., elapsed=0.)
            sample.update(sway=0., bob=0., tilt=0., scale=1.)
            window.face.last_motion_sample = sample
            self.assertTrue(window.face.drop_front_tooth(.484, .615))
            sample['missing_tooth'] = True
            window.face.motion_sample = Mock(return_value=sample)
            start = window.face.tooth_started
            with patch('robot_face_display.time.monotonic', return_value=start+.8):
                falling = window.face.grab().toImage()
            with patch('robot_face_display.time.monotonic', return_value=start+2.):
                finished = window.face.grab().toImage()
            changed = 0
            target = window.face.face_rect()
            for y in range(round(target.y()+target.height()*.74),
                           round(target.y()+target.height()*.86)):
                for x in range(round(target.x()+target.width()*.49),
                               round(target.x()+target.width()*.58)):
                    color = falling.pixelColor(x, y)
                    if color.red() > 170 and color.green() > 140 and color.blue() > 75:
                        changed += color != finished.pixelColor(x, y)
            self.assertGreater(changed, 100, 'The tooth must fall visibly outside the mouth mask')
        finally:
            window.close()

    def test_closed_smile_has_no_original_tooth_highlights(self):
        from PyQt5.QtCore import QRectF
        from PyQt5.QtGui import QPainter, QPixmap
        source = QPixmap(str(DEFAULT_STATES['laughing']))
        canvas = QPixmap(source.size())
        canvas.fill()
        painter = QPainter(canvas)
        motion = FaceMotion()
        sample = motion.sample('laughing', speaking=True, mouth_level=0., elapsed=0.)
        sample.update(sway=0., bob=0., tilt=0., scale=1.)
        try:
            motion.draw(painter, QRectF(canvas.rect()), source, sample, 'laughing')
        finally:
            painter.end()
        image = canvas.toImage()
        white = 0
        for y in range(round(source.height()*.575), round(source.height()*.745)):
            for x in range(round(source.width()*.40), round(source.width()*.60)):
                c = image.pixelColor(x, y)
                white += c.red() > 175 and c.green() > 145 and c.blue() > 85
        self.assertEqual(white, 0, 'Old teeth must not leave floating highlights under the moustache')

    def test_eyes_lead_head_without_frame_rate_dependent_lag(self):
        window = FaceWindow(DEFAULT_FACES, speech_probability=0)
        try:
            face = window.face
            face.set_gaze(1., -.4)
            face.motion_tick_time = 0.
            face.advance_motion(now=1./30)
            self.assertGreater(face.head_gaze_x, 0.)
            self.assertLess(face.head_gaze_x, face.gaze_x*.2)
            sample = face.motion_sample(elapsed=0.)
            self.assertGreater(sample['eye_shift'], .002)
            results = []
            for fps in (30, 60):
                face.head_gaze_x = face.head_gaze_y = 0.
                face.motion_tick_time = 0.
                for tick in range(1, fps+1):
                    face.advance_motion(now=tick/fps)
                results.append(face.head_gaze_x)
            self.assertGreater(results[0], .95)
            self.assertAlmostEqual(*results, places=8)
        finally:
            window.close()

    def test_blinks_have_varied_intervals_fast_closure_and_double_blink(self):
        motion = FaceMotion()
        centers = (3.1, 7.9, 10.5, 16.2, 16.48, 21.8)
        for center in centers:
            self.assertAlmostEqual(motion.blink_amount(center), 1.)
        gaps = [b-a for a, b in zip(centers, centers[1:])]
        self.assertGreater(max(gaps)-min(gaps), 4.)
        self.assertLess(motion.blink_amount(3.05), motion.blink_amount(3.15))
        self.assertLess(motion.blink_amount(0.), .001)

    def test_surprise_and_crying_build_after_expression_change(self):
        motion = FaceMotion()
        for expression in ('astonished', 'crying'):
            initial = motion.sample(expression, elapsed=1., expression_age=0.)
            reacted = motion.sample(expression, elapsed=1., expression_age=.8)
            self.assertLess(initial['mouth_open'], reacted['mouth_open'])
            spoken = motion.sample(expression, elapsed=1., expression_age=0.,
                                   speaking=True, mouth_level=0.)
            self.assertLess(spoken['mouth_open'], .08)

    def test_shared_brows_preserve_original_noses(self):
        from PyQt5.QtCore import QRectF
        from PyQt5.QtGui import QMovie, QPainter, QPixmap
        for state in ('normal', 'laughing', 'crying', 'astonished', 'error'):
            movie = QMovie(str(DEFAULT_STATES[state])); movie.jumpToFrame(0)
            source = movie.currentPixmap(); original = source.toImage()
            canvas = QPixmap(source.size()); canvas.fill()
            motion = FaceMotion()
            sample = motion.sample(state, elapsed=1.)
            sample.update(sway=0., bob=0., tilt=0., scale=1., eye_shift=0.)
            painter = QPainter(canvas)
            motion.draw(painter, QRectF(canvas.rect()), source, sample, state)
            painter.end(); image = canvas.toImage()
            for y in range(round(source.height()*.39), round(source.height()*.55)):
                for x in range(round(source.width()*.46), round(source.width()*.54)):
                    color = original.pixelColor(x, y)
                    if color.red() > 150 and 90 < color.green() < 210 and color.red()-color.blue() > 45:
                        self.assertEqual(color, image.pixelColor(x, y), (state, x, y))

    def test_angry_source_brows_do_not_remain_below_unified_brows(self):
        from PyQt5.QtCore import QRectF
        from PyQt5.QtGui import QMovie, QPainter, QPixmap
        movie = QMovie(str(DEFAULT_STATES['error'])); movie.jumpToFrame(0)
        source = movie.currentPixmap()
        canvas = QPixmap(source.size()); canvas.fill()
        motion = FaceMotion()
        sample = motion.sample('error', elapsed=1.)
        sample.update(sway=0., bob=0., tilt=0., scale=1.)
        painter = QPainter(canvas)
        motion.draw(painter, QRectF(canvas.rect()), source, sample, 'error')
        painter.end(); image = canvas.toImage()
        gray = 0
        for y in range(round(source.height()*.318), round(source.height()*.335)):
            for x in range(round(source.width()*.35), round(source.width()*.65)):
                c = image.pixelColor(x, y)
                gray += 25 < max(c.red(), c.green(), c.blue()) < 150 and max(c.red(), c.green(), c.blue())-min(c.red(), c.green(), c.blue()) < 40
        self.assertEqual(gray, 0)


if __name__ == '__main__':
    unittest.main()
