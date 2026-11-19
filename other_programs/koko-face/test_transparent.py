"""Verify alpha outside the character, desktop hit regions and both launch modes."""
import os
import sys
import unittest
from unittest.mock import Mock, patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtGui import QPainter, QPixmap
from robot_face_display import QApplication, FaceWindow, DEFAULT_FACES, parse_arguments


class TransparentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = FaceWindow(DEFAULT_FACES, speech_probability=0, transparent_background=True)
        self.window.voice.try_play = Mock(return_value=False)
        self.window.resize(960, 540)
        self.window.pointer_timer.stop(); self.window.companion_timer.stop()
        self.window.face.animation_timer.stop()
        self.window.show(); self.app.processEvents()
        self.window.face.previous_pixmap = None
        self.window.face.bubble_text = ''

    def tearDown(self):
        self.window.close()

    def scene(self):
        return self.window.face.render_scene().toImage()

    def test_background_is_transparent_and_character_remains_opaque(self):
        self.assertTrue(self.window.testAttribute(Qt.WA_TranslucentBackground))
        self.assertTrue(self.window.windowFlags() & Qt.FramelessWindowHint)
        for state in ('normal', 'crying', 'radiant', 'astonished'):
            self.window.show_state(state); self.window.face.previous_pixmap = None
            image = self.scene()
            self.assertEqual(image.pixelColor(10, 10).alpha(), 0, state)
            self.assertEqual(image.pixelColor(480, 80).alpha(), 0, state)
            self.assertEqual(image.pixelColor(480, 270).alpha(), 255, state)
            self.assertEqual(image.pixelColor(300, 530).alpha(), 0, state)

    def test_input_mask_leaves_desktop_outside_character_clickable(self):
        self.window.face.repaint()
        self.app.processEvents()
        mask = self.window.mask()
        self.assertFalse(mask.isEmpty())
        self.assertFalse(mask.contains(QPoint(10, 10)))
        self.assertTrue(mask.contains(QPoint(480, 270)))
        self.assertTrue(mask.contains(QPoint(480, 85)), "The forehead remains touchable")

    def test_split_and_merge_sprites_preserve_alpha(self):
        self.window.start_split_play()
        first = self.window.face.split_play.faces[0]
        sample = self.window.face.motion_sample()
        frame = self.window.movie.currentPixmap()
        for blend in (0., .5, 1.):
            first['merge_blend'] = blend
            self.window.face.split_sprite_cache.clear()
            image = self.window.face.split_sprite(first, frame, sample).toImage()
            self.assertEqual(image.pixelColor(0, 0).alpha(), 0)
            self.assertEqual(image.pixelColor(320, 180).alpha(), 255)
        image = self.scene()
        self.assertEqual(image.pixelColor(5, 5).alpha(), 0)

    def test_original_mode_still_has_an_opaque_black_background(self):
        black = FaceWindow(DEFAULT_FACES, speech_probability=0)
        try:
            black.resize(960, 540); black.show(); self.app.processEvents()
            image = black.face.grab().toImage()
            self.assertFalse(black.testAttribute(Qt.WA_TranslucentBackground))
            self.assertEqual(image.pixelColor(5, 5).alpha(), 255)
            self.assertEqual(image.pixelColor(5, 5).red(), 0)
        finally:
            black.close()

    def test_cli_transparency_is_opt_in(self):
        for arguments, transparent in (([], False), (['--transparent'], True)):
            with patch.object(sys, 'argv', ['koko-face', *arguments]):
                self.assertEqual(parse_arguments().transparent, transparent)


if __name__ == '__main__':
    unittest.main()
