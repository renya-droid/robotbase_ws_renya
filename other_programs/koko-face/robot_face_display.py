#!/usr/bin/env python3
"""Show an animated robot face while velocity commands indicate motion."""

import argparse
import math
import os
import random
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from touch_voice import TouchVoice
from hand_gesture import HandGesture
from PyQt5.QtCore import pyqtSignal, QPointF, QSize, QRect, QRectF, QTimer, Qt
from PyQt5.QtGui import QBitmap, QColor, QCursor, QFont, QFontMetrics, QMovie, QPainter, QPainterPath, QPixmap, QRegion, QTransform
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QSizePolicy
from face_motion import FaceMotion, interpolate
from mouth_art import FRONT_TOOTH, draw_tooth
from expression_art import EXTRA_EXPRESSIONS, NATIVE_EXPRESSIONS
from butterfly import Butterfly
from split_play import SplitPlay
from companion_play import CompanionPlay


ASSET_DIRECTORY = Path(__file__).resolve().parent / 'assets'
DEFAULT_STATES = {
    name: ASSET_DIRECTORY / f'bearded_face_{name}.gif'
    for name in (
        'normal', 'smile', 'scanning', 'success', 'surprise', 'thinking',
        'moving', 'happy', 'sleep', 'troubled', 'error', 'speaking', 'licking', 'nodding', 'playful',
    )
}
DEFAULT_STATES['laughing'] = ASSET_DIRECTORY / 'bearded_face_laughing.png'
DEFAULT_STATES['happy'] = DEFAULT_STATES['laughing']
for expression in ('crying', 'shy', 'astonished', 'radiant'):
    DEFAULT_STATES[expression] = ASSET_DIRECTORY / f'bearded_face_{expression}.png'
for expression in NATIVE_EXPRESSIONS:
    DEFAULT_STATES[expression] = DEFAULT_STATES['normal']
DEFAULT_FACES = tuple(
    DEFAULT_STATES[name] for name in
    ('normal', 'smile', 'surprise', 'thinking', 'happy', 'troubled', 'speaking',
     'nodding', 'playful', 'crying', 'shy', 'astonished', 'radiant')
)
DEFAULT_FACE = DEFAULT_FACES[0]
DEFAULT_WINKS = {
    'left': ASSET_DIRECTORY / 'bearded_face_wink_left.gif',
    'right': ASSET_DIRECTORY / 'bearded_face_wink_right.gif',
}
DEFAULT_TURN_CONTROL = ASSET_DIRECTORY / 'cat_face_turn_control.gif'
EXPRESSION_VOICE_CUES = {
    'smile': 'cavity', 'happy': 'cavity', 'laughing': 'cavity',
    'radiant': 'radiant', 'shy': 'shy', 'crying': 'crying',
}


def stain_teeth(pixmap, bounds):
    """Render subtle staining on pale tooth pixels, preserving gaps and outlines."""
    image = pixmap.toImage()
    mask = QRegion()
    for y in range(bounds.top(), bounds.bottom() + 1):
        start = None
        for x in range(bounds.left(), bounds.right() + 2):
            pale = False
            if x <= bounds.right():
                color = image.pixelColor(x, y)
                pale = (color.alpha() > 150 and color.red() > 185
                        and color.green() > 160 and color.blue() > 100)
            if pale and start is None:
                start = x
            elif not pale and start is not None:
                mask += QRegion(start, y, x - start, 1)
                start = None
    result = pixmap.copy()
    painter = QPainter(result)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setClipRegion(mask)
    painter.fillRect(bounds, QColor(177, 135, 40, 45))
    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor(143, 102, 44, 65))
    for x, y, width, height in ((.13, .72, .08, .16), (.36, .25, .06, .12),
                                 (.72, .73, .07, .13), (.90, .33, .05, .16)):
        painter.drawEllipse(QRectF(bounds.x() + x * bounds.width(),
                                  bounds.y() + y * bounds.height(),
                                  width * bounds.width(), height * bounds.height()))
    painter.end()
    return result


def is_moving(message, linear_threshold=0.02, angular_threshold=0.05):
    """Return whether a Twist contains a meaningful motion command."""
    linear = max(abs(message.linear.x), abs(message.linear.y), abs(message.linear.z))
    angular = max(abs(message.angular.x), abs(message.angular.y), abs(message.angular.z))
    return linear >= linear_threshold or angular >= angular_threshold


class FaceLabel(QLabel):
    touched = pyqtSignal(float, float)
    dragged = pyqtSignal(float, float)
    drag_finished = pyqtSignal()

    def __init__(self, parent=None, transparent_background=False):
        super().__init__(parent)
        self.transparent_background = transparent_background
        self.press_position = None
        self.is_dragging = False
        self.pointer_last = None
        self.pointer_time = 0.
        self.play = CompanionPlay()
        self.gaze_x = 0.0
        self.gaze_y = 0.0
        self.render_scale = 1.0
        self.motion = FaceMotion()
        self.talk_active = False
        self.talk_level = None
        self._composed_key = None
        self._composed_frame = None
        self.last_motion_sample = None
        self.last_interaction = time.monotonic()
        self.expression_started = self.last_interaction
        self.motion_tick_time = self.last_interaction
        self.head_gaze_x = self.head_gaze_y = 0.
        self.return_sample = None
        self.return_started = None
        self.bubble_text = ''
        self.bubble_until = 0.
        self.hands = HandGesture(ASSET_DIRECTORY / 'hand_open.png')
        self.smile_teeth = QPixmap(str(ASSET_DIRECTORY / 'gap_tooth_smile.png'))
        if self.smile_teeth.isNull():
            raise RuntimeError('笑顔の歯の画像を読み込めません')
        # Exclude transparent padding and faint generation artifacts.
        self.smile_teeth = self.smile_teeth.copy(QRect(100, 200, 1972, 360))
        self.smile_teeth = self.smile_teeth.scaledToWidth(384, Qt.SmoothTransformation)
        self.smile_teeth = stain_teeth(self.smile_teeth, self.smile_teeth.rect())
        self.previous_pixmap = None
        self.transition_started = 0.0
        self.tooth_missing = False
        self.tooth_started = None
        self.butterfly = Butterfly()
        self.split_play = SplitPlay()
        self.split_sources = {}
        self.split_sprite_cache = {}
        self.expression_override = None
        self.tooth_timer = QTimer(self)
        self.tooth_timer.setInterval(16)
        self.tooth_timer.timeout.connect(self.update)
        laughing_image = QPixmap(str(DEFAULT_STATES['laughing'])).toImage()
        self.stained_laughing = stain_teeth(QPixmap.fromImage(laughing_image), QRect(
            round(laughing_image.width() * .410), round(laughing_image.height() * .577),
            round(laughing_image.width() * .177), round(laughing_image.height() * .069)))
        self.mouth_color = laughing_image.pixelColor(
            round(laughing_image.width() * .55), round(laughing_image.height() * .65))
        # Match the existing gap's silhouette, including its rounded bottom,
        # while keeping the surrounding teeth and moustache intact.
        self.original_gap = QRegion()
        for y in range(round(laughing_image.height() * .59), round(laughing_image.height() * .645)):
            for x in range(round(laughing_image.width() * .499), round(laughing_image.width() * .535)):
                color = laughing_image.pixelColor(x, y)
                if max(color.red(), color.green(), color.blue()) < 80:
                    self.original_gap += QRegion(x, y, 1, 1)
        self.setAttribute(Qt.WA_OpaquePaintEvent, not transparent_background)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setMinimumSize(1, 1)
        self.animation_timer = QTimer(self)
        self.animation_timer.setInterval(33)
        self.animation_timer.timeout.connect(self.advance_motion)
        self.animation_timer.start()

    def advance_motion(self, now=None):
        now = time.monotonic() if now is None else now
        dt = max(0., now-self.motion_tick_time)
        self.motion_tick_time = now
        self.play.advance(now)
        # The eyes orient first; the slower head catches up over a few frames.
        amount = 1.-math.exp(-dt/.24)
        self.head_gaze_x += (self.gaze_x-self.head_gaze_x)*amount
        self.head_gaze_y += (self.gaze_y-self.head_gaze_y)*amount
        target = self.face_rect()
        if self.split_play.advance(now, (target.width()/max(1, self.width()),
                                         target.height()/max(1, self.height()))):
            self.last_interaction = now
            self.return_sample = None
            self.last_motion_sample = None
            self.bubble_text = ''
        self.update()

    def set_gaze(self, x, y):
        self.gaze_x = max(-1.0, min(1.0, x))
        self.gaze_y = max(-1.0, min(1.0, y))
        self.update()

    def face_rect(self):
        """Use one fixed canvas for every expression, at 195 percent of native size."""
        size = QSize(1872, 1053).scaled(
            min(1872, self.width()), min(1053, self.height()), Qt.KeepAspectRatio)
        return QRect((self.width() - size.width()) // 2,
                     (self.height() - size.height()) // 2,
                     size.width(), size.height())

    def paintEvent(self, event):
        if not self.transparent_background:
            painter = QPainter(self)
            try:
                self.paint_scene(painter)
            finally:
                painter.end()
            return
        scene = self.render_scene()
        painter = QPainter(self)
        painter.setCompositionMode(QPainter.CompositionMode_Source)
        painter.drawPixmap(0, 0, scene)
        painter.end()
        mask = self.desktop_input_region(scene)
        window = self.window()
        if not mask.isEmpty() and mask != window.mask():
            window.setMask(mask.translated(self.mapTo(window, self.rect().topLeft())))

    def render_scene(self):
        scene = QPixmap(self.size()); scene.fill(Qt.transparent)
        painter = QPainter(scene)
        try:
            self.paint_scene(painter)
        finally:
            painter.end()
        scene.setMask(scene.createMaskFromColor(QColor(Qt.black), Qt.MaskInColor))
        return scene

    def desktop_input_region(self, scene):
        mask = QRegion(QBitmap.fromImage(scene.toImage().createAlphaMask()))
        target = self.face_rect()
        if self.split_play.active:
            for face in self.split_play.faces:
                if face['opacity'] < .005:
                    continue
                width, height = target.width()*face['scale'], target.height()*face['scale']
                transform = QTransform()
                transform.translate(face['x']*self.width(), face['y']*self.height())
                transform.rotate(face['angle'])
                outline = transform.map(self.motion.face_outline(QRectF(-width/2, -height/2, width, height)))
                mask |= QRegion(outline.toFillPolygon().toPolygon())
        elif self.last_motion_sample is not None:
            outline = self.motion.transform(target, self.last_motion_sample).map(self.motion.face_outline(target))
            mask |= QRegion(outline.toFillPolygon().toPolygon())
        return mask

    def paint_scene(self, painter):
        """Animate the whole face and its eyes/mouth, including static PNGs."""
        movie = self.movie()
        if movie is None:
            return
        pixmap = movie.currentPixmap()
        if Path(movie.fileName()).name == 'bearded_face_laughing.png':
            pixmap = self.stained_laughing
        if not self.transparent_background:
            painter.fillRect(self.rect(), Qt.black)
        if pixmap.isNull():
            return
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        target = self.face_rect()
        sample = self.motion_sample()
        self.last_motion_sample = sample
        if self.split_play.active:
            base_frame = self.compose_frame(movie, pixmap)
            self.split_play.paint(painter, self.rect(), target,
                                  lambda face: self.split_sprite(face, base_frame, sample),
                                  self.motion.draw_contour)
            self.paint_bubble(painter, target, sample)
            return
        blend = min(1.0, (time.monotonic() - self.transition_started) / 0.18)
        if self.previous_pixmap is not None and blend < 1.0:
            self.motion.draw(painter, target, self.previous_pixmap, sample,
                             ('previous', self.previous_pixmap.cacheKey(), sample['expression']))
            painter.setOpacity(blend)
        else:
            self.previous_pixmap = None
        frame = self.compose_frame(movie, pixmap)
        self.motion.draw(painter, target, frame, sample,
                         (movie.fileName(), movie.currentFrameNumber()))
        painter.setOpacity(1.0)
        painter.save()
        painter.setTransform(self.motion.transform(target, sample), True)
        self.hands.paint(painter, target)
        painter.restore()
        self.paint_falling_tooth(painter, target)
        self.paint_bubble(painter, target, sample)
        transform = self.motion.transform(target, sample)
        nose = transform.map(QPointF(target.x()+target.width()*.5, target.y()+target.height()*.495))
        mouth = transform.map(QPointF(target.x()+target.width()*.5,
                                     target.y()+target.height()*interpolate(.69, sample['points'])))
        self.butterfly.paint(painter, self.rect(), nose_anchor=nose, mouth_anchor=mouth)
        self.play.paint(painter, self.rect(), time.monotonic(), sample)

    def split_sprite(self, face, base_frame, base_sample):
        """Render each companion's own emotion, clock and gaze without audio."""
        now = time.monotonic()
        emotion = face.get('social_emotion') if now < face.get('social_until', 0.) else None
        emotion = emotion or face['emotion']
        cache_key = (emotion, face['emotion_since'], int(now*15),
                     round(face['merge_blend'], 3))
        cached = self.split_sprite_cache.get(face['id'])
        if cached is not None and cached[0] == cache_key:
            return cached[1]

        def render(emotion):
            if emotion not in self.split_sources:
                source = QPixmap(str(DEFAULT_STATES[emotion]))
                if source.isNull():
                    raise RuntimeError(f'分裂用の表情を読み込めません: {emotion}')
                self.split_sources[emotion] = source
            source = self.split_sources[emotion]
            t = now-self.motion.started+face['phase']
            gaze = (face.get('social_gaze', 0.) if now < face.get('social_until', 0.)
                    else max(-1., min(1., face['vx']*4.)))
            pose = self.motion.sample(emotion, gaze_x=gaze,
                                      gaze_y=.25*math.sin(t*.7), elapsed=t,
                                      expression_age=max(0., now-face['emotion_since']))
            pose.update(sway=0., bob=0., tilt=0., scale=1., idle_blend=0.)
            pose['transparent_background'] = self.transparent_background
            image = QPixmap(640, 360); image.fill(Qt.transparent if self.transparent_background else Qt.black)
            draw = QPainter(image)
            self.motion.draw(draw, QRectF(image.rect()), source, pose, ('split', emotion))
            draw.end()
            image.setMask(image.createMaskFromColor(QColor(Qt.black), Qt.MaskInColor))
            return image

        image = render(emotion)
        layers = []
        age = now-face['emotion_since']
        if face['previous_emotion'] is not None and age < .24:
            layers.append((render(face['previous_emotion']), image, max(0., age/.24)))
        if face['merge_blend'] > 0.:
            original = QPixmap(640, 360); original.fill(Qt.transparent if self.transparent_background else Qt.black)
            draw = QPainter(original)
            pose = base_sample.copy()
            pose.update(sway=0., bob=0., tilt=0., scale=1., idle_blend=0.)
            self.motion.draw(draw, QRectF(original.rect()), base_frame, pose,
                             ('split-original', self.movie().fileName()))
            draw.end()
            original.setMask(original.createMaskFromColor(QColor(Qt.black), Qt.MaskInColor))
            layers.append((image, original, face['merge_blend']))
        for old, new, amount in layers:
            mixed = QPixmap(640, 360); mixed.fill(Qt.transparent if self.transparent_background else Qt.black)
            draw = QPainter(mixed)
            draw.setOpacity(1. if self.transparent_background else 1.-amount)
            draw.drawPixmap(0, 0, old)
            if not self.transparent_background:
                draw.setCompositionMode(QPainter.CompositionMode_Plus)
            draw.setOpacity(amount); draw.drawPixmap(0, 0, new)
            draw.end()
            if not self.transparent_background:
                mixed.setMask(mixed.createMaskFromColor(QColor(Qt.black), Qt.MaskInColor))
            image = mixed
        # Only retain current companions, including during repeated splitting.
        live_ids = {item['id'] for item in self.split_play.faces}
        self.split_sprite_cache = {key: value for key, value in self.split_sprite_cache.items() if key in live_ids}
        self.split_sprite_cache[face['id']] = (cache_key, image)
        return image

    def say(self, text, duration=3.):
        self.bubble_text = text
        self.bubble_until = time.monotonic()+duration

    def paint_bubble(self, painter, target, sample):
        if not self.bubble_text or time.monotonic() >= self.bubble_until:
            return
        painter.save()
        font = QFont('Sans Serif', max(10, round(target.height()*.022)))
        painter.setFont(font)
        metrics = QFontMetrics(font)
        width = min(self.width()-24, metrics.horizontalAdvance(self.bubble_text)+24)
        height = metrics.height()+20
        anchor = self.motion.transform(target, sample).map(
            QPointF(target.x()+target.width()*.72, target.y()+target.height()*.16))
        x = max(12., min(self.width()-width-12., anchor.x()))
        y = max(12., min(self.height()-height-12., anchor.y()))
        rect = QRectF(x, y, width, height)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QColor('#b88f6c'))
        painter.setBrush(QColor('#fff4dc'))
        painter.drawRoundedRect(rect, 12., 12.)
        painter.setPen(QColor('#4b342b'))
        painter.drawText(rect, Qt.AlignCenter, self.bubble_text)
        painter.restore()

    def motion_sample(self, elapsed=None):
        movie = self.movie()
        expression = (Path(movie.fileName()).stem.removeprefix('bearded_face_')
                      if movie is not None else 'normal')
        expression = self.expression_override or expression
        sample = self.motion.sample(expression, self.gaze_x, self.gaze_y,
                                    self.talk_active, self.talk_level, elapsed,
                                    idle_seconds=time.monotonic()-self.last_interaction,
                                    head_gaze=(self.head_gaze_x, self.head_gaze_y),
                                    expression_age=(self.butterfly.elapsed()-Butterfly.INHALE
                                                    if expression == 'sneezing' and self.butterfly.active
                                                    else time.monotonic()-self.expression_started))
        if self.butterfly.active:
            age = self.butterfly.elapsed()
            point = self.butterfly.chase_position(age)
            blend = self.motion.smoothstep(age/1.2)
            sample['scale'] = (1.-.38*blend)*(1.+.002*math.sin(age*2.))
            sample['sway'] = blend*(point.x()-.5)*.42
            sample['bob'] = blend*(point.y()-.5)*.28
            sample['tilt'] += blend*self.gaze_x*3.
            sample['idle_blend'] = blend
            sample['cross_eye'] = (self.butterfly.landing_blend(age)
                                   *(1.-self.motion.smoothstep((age-Butterfly.INHALE)/.4)))
            if expression == 'sneezing':
                burst = math.exp(-((age-Butterfly.SNEEZE)/.115)**2)
                sample['bob'] += .022*burst
        sample['transparent_background'] = self.transparent_background
        sample['missing_tooth'] = self.tooth_missing
        if self.return_sample is not None:
            amount = min(1., max(0., (time.monotonic()-self.return_started)/1.5))
            amount = amount**2 * (3.-2.*amount)
            for key in ('sway', 'bob', 'scale'):
                sample[key] = self.return_sample[key]*(1.-amount) + sample[key]*amount
            if amount >= 1.:
                self.return_sample = None
        self.play.apply_pose(sample, time.monotonic(), self.rect(), self.face_rect())
        return sample

    def note_interaction(self):
        if self.last_motion_sample is not None and self.last_motion_sample.get('idle_blend', 0.) > 0.:
            self.return_sample = self.last_motion_sample.copy()
            self.return_started = time.monotonic()
        self.last_interaction = time.monotonic()

    def compose_frame(self, movie, pixmap):
        key = (movie.fileName(), movie.currentFrameNumber(), self.tooth_missing)
        if self._composed_key == key:
            return self._composed_frame
        frame = pixmap.copy()
        painter = QPainter(frame)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        target = frame.rect()
        if Path(movie.fileName()).name == 'bearded_face_laughing.png':
            painter.save()
            painter.setClipRegion(self.original_gap)
            painter.fillRect(pixmap.rect(), self.mouth_color)
            painter.restore()
        # Keep the original GIF animation and place teeth only inside smiles.
        teeth_rects = {
            'bearded_face_happy.gif': QRectF(441, 350, 77, 20),
            'bearded_face_smile.gif': QRectF(445, 345, 69, 18),
        }
        teeth_rect = teeth_rects.get(Path(movie.fileName()).name)
        if teeth_rect is not None:
            painter.drawPixmap(QRectF(
                target.x() + teeth_rect.x() * target.width() / pixmap.width(),
                target.y() + teeth_rect.y() * target.height() / pixmap.height(),
                teeth_rect.width() * target.width() / pixmap.width(),
                teeth_rect.height() * target.height() / pixmap.height()),
                self.smile_teeth, QRectF(self.smile_teeth.rect()))
        painter.end()
        self._composed_key = key
        self._composed_frame = frame
        return frame

    # Coordinates are relative to the adopted laughing image, independent of screen size.
    FRONT_TOOTH = FRONT_TOOTH

    def drop_front_tooth(self, x, y):
        movie = self.movie()
        if (movie is None or Path(movie.fileName()).name != 'bearded_face_laughing.png'
                or self.tooth_missing or not self.FRONT_TOOTH.contains(x, y)
                or (self.last_motion_sample is not None
                    and self.last_motion_sample.get('mouth_open', 1.) < .15)):
            return False
        target = self.face_rect()
        sample = self.last_motion_sample or self.motion_sample()
        source = self.FRONT_TOOTH
        center = QPointF(target.x()+source.center().x()*target.width(),
                         target.y()+interpolate(source.center().y(), sample['points'])*target.height())
        self.tooth_origin = self.motion.transform(target, sample).map(center)
        self.tooth_size = (source.width()*target.width()*sample['scale'],
                           source.height()*target.height()*max(.75, sample['mouth_open'])*sample['scale'])
        self.tooth_missing = True
        self.tooth_started = time.monotonic()
        self.tooth_timer.start()
        self.update()
        return True

    def reset_tooth(self):
        self.tooth_timer.stop()
        self.tooth_missing = False
        self.tooth_started = None
        self._composed_key = None
        self.last_motion_sample = None

    def paint_falling_tooth(self, painter, target, elapsed=None):
        """Paint in window coordinates, above the face and outside its mouth mask."""
        if not self.tooth_missing or self.tooth_started is None:
            return
        elapsed = time.monotonic()-self.tooth_started if elapsed is None else elapsed
        if elapsed >= 1.6:
            self.tooth_timer.stop()
            return
        width, height = self.tooth_size
        painter.save()
        painter.translate(self.tooth_origin.x()+target.width()*.07*elapsed,
                          self.tooth_origin.y()+target.height()*.28*elapsed**2)
        painter.rotate(220.*elapsed)
        draw_tooth(painter, QRectF(-width/2, -height/2, width, height))
        painter.restore()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.press_position = (event.x(), event.y())
            self.is_dragging = False
            self.pointer_last = self.press_position
            self.pointer_time = time.monotonic()
            self.play.food_held = self.play.food_hit(event.x()/max(1, self.width()),
                                                   event.y()/max(1, self.height()),
                                                   self.width()/max(1, self.height()))
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.press_position is not None and event.buttons() & Qt.LeftButton:
            now = time.monotonic()
            if self.play.food_held:
                self.play.food = (event.x()/max(1, self.width()), event.y()/max(1, self.height()))
                self.update()
            else:
                distance = math.hypot(event.x()-self.press_position[0], event.y()-self.press_position[1])
                if distance >= QApplication.startDragDistance():
                    self.is_dragging = True
                if self.is_dragging:
                    if self.split_play.active:
                        self.split_play.drag_to(self.press_position[0]/max(1, self.width()),
                                                self.press_position[1]/max(1, self.height()),
                                                event.x()/max(1, self.width()), event.y()/max(1, self.height()), now)
                    else:
                        self.dragged.emit((event.x()-self.pointer_last[0])/max(1, self.width()),
                                          (event.y()-self.pointer_last[1])/max(1, self.height()))
            self.pointer_last = (event.x(), event.y())
            self.pointer_time = now
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.press_position is not None:
            if self.play.food_held:
                self.play.food_held = False
                self.play.food = (event.x()/max(1, self.width()), event.y()/max(1, self.height()))
                self.window().finish_food()
            elif self.is_dragging:
                if self.split_play.active:
                    self.split_play.release_drag(time.monotonic())
                else:
                    self.drag_finished.emit()
            else:
                self.touched.emit(event.x()/max(1, self.width()), event.y()/max(1, self.height()))
            self.press_position = None
            self.is_dragging = False
        super().mouseReleaseEvent(event)


class FaceWindow(QMainWindow):
    def __init__(self, image_paths, screen_index=0, stop_delay=0.8,
                 wink_paths=None, state_paths=None, turn_control_path=None,
                 voice_directory=None, speech_probability=.3, transparent_background=False):
        super().__init__()
        self.transparent_background = transparent_background
        self.stop_delay = stop_delay
        self.last_motion_time = None
        if isinstance(image_paths, (str, Path)):
            image_paths = [image_paths]
        self.image_paths = [Path(path) for path in image_paths]
        if not self.image_paths:
            raise RuntimeError('表示する顔アニメーションがありません')
        self.current_animation = 0
        self.current_state = 'normal'
        self.speech_active = False
        self.speech_resume_state = 'normal'
        self.voice = TouchVoice(voice_directory or ASSET_DIRECTORY / 'voice',
                                probability=speech_probability, parent=self)
        self.voice.active_changed.connect(self.set_speaking)
        self.voice.mouth_level.connect(self.set_mouth_level)
        self.touch_times = []
        self.wink_paths = {
            side: Path(path) for side, path in (wink_paths or DEFAULT_WINKS).items()
        }
        self.state_paths = {
            state: Path(path) for state, path in (state_paths or DEFAULT_STATES).items()
        }
        self.turn_control_path = Path(turn_control_path) if turn_control_path else None
        self.drag_frame = None
        self.drag_resume_state = 'normal'
        self.direction_active = False
        self.direction_resume_state = 'normal'
        self.direction_suspended_until = 0.0

        self.setWindowTitle('ココちゃん走行表示')
        if transparent_background:
            self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool)
        self.setAttribute(Qt.WA_TranslucentBackground, transparent_background)
        self.setStyleSheet('background: transparent;' if transparent_background else 'background: black;')
        self.setCursor(Qt.ArrowCursor)

        self.face = FaceLabel(self, transparent_background=transparent_background)
        self.face.setAlignment(Qt.AlignCenter)
        self.face.setAttribute(Qt.WA_TranslucentBackground, transparent_background)
        self.face.setStyleSheet('background: transparent;' if transparent_background else 'background: black;')
        self.face.touched.connect(self.handle_face_touch)
        self.face.dragged.connect(self.handle_drag)
        self.face.drag_finished.connect(self.finish_drag)
        self.setCentralWidget(self.face)

        self.gaze_x = 0.0
        self.gaze_y = 0.0
        self.last_pointer_position = None
        self.pointer_timer = QTimer(self)
        self.pointer_timer.setInterval(16)
        self.pointer_timer.timeout.connect(self.update_pointer_gaze)
        self.pointer_timer.start()

        self.return_timer = QTimer(self)
        self.return_timer.setInterval(25)
        self.return_timer.timeout.connect(self._return_drag_to_center)

        self.tooth_reaction_started = None
        self.tooth_reaction_stage = 0
        self.idle_origin_state = None
        self.idle_phase = None
        self.butterfly_resume_state = None
        self.butterfly_automatic = False
        self.last_butterfly_visit = -float('inf')
        self.companion_timer = QTimer(self)
        self.companion_timer.setInterval(150)
        self.companion_timer.timeout.connect(self.update_companion)
        self.companion_timer.start()

        self.movie = None
        self.show_state('normal')

        screens = QApplication.screens()
        selected = max(0, min(screen_index, len(screens) - 1))
        self.setGeometry(screens[selected].geometry())

    def update_pointer_gaze(self, cursor_position=None):
        """Ease the face toward the current pointer position."""
        if self.face.split_play.active:
            return
        if self.face.play.mode is not None:
            if self.face.play.mode == 'food':
                self.face.set_gaze((self.face.play.food[0]-.5)*2., (self.face.play.food[1]-.5)*2.)
            return
        if self.face.butterfly.active:
            point = self.face.butterfly.chase_position(self.face.butterfly.elapsed())
            target_x, target_y = (point.x()-.5)*2., (point.y()-.5)*2.
            self.gaze_x += (target_x-self.gaze_x)*.24
            self.gaze_y += (target_y-self.gaze_y)*.24
            self.face.set_gaze(self.gaze_x, self.gaze_y)
            return
        if cursor_position is None:
            cursor_position = QCursor.pos()
        if self.last_pointer_position is not None and (
                cursor_position-self.last_pointer_position).manhattanLength() > 2:
            self.face.note_interaction()
        self.last_pointer_position = QPointF(cursor_position).toPoint()
        local = self.face.mapFromGlobal(cursor_position)
        width = max(1, self.face.width())
        height = max(1, self.face.height())
        target_x = max(-1.0, min(1.0, (local.x() / width - 0.5) * 2.0))
        target_y = max(-1.0, min(1.0, (local.y() / height - 0.5) * 2.0))
        smoothing = 0.24
        self.gaze_x += (target_x - self.gaze_x) * smoothing
        self.gaze_y += (target_y - self.gaze_y) * smoothing

        if time.monotonic() < self.direction_suspended_until:
            self.face.set_gaze(0.0, self.gaze_y)
            return

        if self.speech_active or self.turn_control_path is None:
            self.face.set_gaze(self.gaze_x, self.gaze_y)
            return

        if abs(self.gaze_x) >= 0.10:
            if not self.direction_active:
                self.direction_resume_state = self.current_state
                self._start_movie(self.turn_control_path)
                self.movie.setPaused(True)
                self.direction_active = True
            frame_count = self.movie.frameCount()
            frame = round((self.gaze_x + 1.0) * 0.5 * (frame_count - 1))
            self.movie.jumpToFrame(max(0, min(frame_count - 1, frame)))
            self.face.set_gaze(0.0, self.gaze_y)
        else:
            if self.direction_active:
                resume_state = self.direction_resume_state
                self.direction_active = False
                self.show_state(resume_state)
            self.face.set_gaze(0.0, self.gaze_y)

    def show_animation(self, index):
        """Start one animation loop by index."""
        if self.speech_active:
            return
        image_path = self.image_paths[index]
        self.current_animation = index
        for state, path in self.state_paths.items():
            if path == image_path:
                self.show_state(state)
                return
        self.face.hands.set_gripping(False)
        self._start_movie(image_path)

    def show_state(self, state):
        """Show a named robot or interaction state."""
        aliases = {
            'idle': 'normal', 'complete': 'success', 'completed': 'success',
            'warning': 'error', 'angry': 'error', 'recognizing': 'scanning',
            'recognising': 'scanning', 'talking': 'speaking',
            'lick': 'licking', 'tongue': 'licking',
            'smirk': 'smile', 'grinning': 'smile',
            'cry': 'crying', 'bashful': 'shy', 'surprised': 'astonished',
            'beaming': 'radiant',
        }
        state = aliases.get(state.strip().lower(), state.strip().lower())
        if state in ('butterfly', 'chase_butterfly'):
            return self.recall_butterfly()
        if state not in self.state_paths:
            return False
        self.face.play.stop()
        self.face.split_play.stop()
        if self.face.butterfly.active:
            self.stop_butterfly(restore=False)
        if state != self.current_state:
            self.tooth_reaction_started = None
            self.idle_origin_state = None
            self.idle_phase = None
            self.face.bubble_text = ''
        if self.speech_active and state != 'speaking':
            self.speech_resume_state = state
            return True
        self.face.hands.set_gripping(state == 'smile')
        state_path = self.state_paths[state]
        if (state != self.current_state or self.movie is None or
                Path(self.movie.fileName()) != state_path):
            entering_expression = state != self.current_state
            self.current_state = state
            self._start_movie(state_path)
            if entering_expression and state in EXPRESSION_VOICE_CUES:
                self.voice.try_play(EXPRESSION_VOICE_CUES[state])
        return True

    def _show_reaction(self, state):
        """Change visual emotion without interrupting or starting audio."""
        if self.speech_active:
            self.speech_resume_state = state
        if state != self.current_state:
            self.current_state = state
            self.face.hands.set_gripping(state == 'smile')
            self._start_movie(self.state_paths[state])

    def start_split_play(self):
        if self.current_state == 'moving':
            return False
        self.stop_butterfly()
        self.face.play.stop()
        self.tooth_reaction_started = None
        self.idle_origin_state = self.idle_phase = None
        self.direction_active = False
        self.face.reset_tooth()
        self.face.note_interaction()
        self.face.return_sample = None
        self.face.split_sprite_cache.clear()
        self.face.split_play.start()
        self.face.say('タッチで分裂！ 放っておくと合体するよ', 4.)
        return True

    def start_butterfly(self, now=None):
        now = time.monotonic() if now is None else now
        if (self.face.butterfly.active or self.face.split_play.active or self.speech_active or
                self.face.play.mode is not None or
                self.tooth_reaction_started is not None or self.current_state == 'moving' or
                any(state not in self.state_paths for state in ('curious', 'excited', 'relieved'))):
            return False
        self.butterfly_resume_state = self.idle_origin_state or self.current_state
        self.butterfly_automatic = False
        self.idle_origin_state = self.idle_phase = None
        self.last_butterfly_visit = now
        self.face.butterfly.start(now)
        self.direction_active = False
        self.face.return_sample = None
        self._show_reaction('curious')
        self.face.say('あっ、ちょうちょ！', 2.5)
        return True

    def recall_butterfly(self):
        """An explicit request restarts a visit, independently of idle timing."""
        if self.face.butterfly.active:
            self.stop_butterfly()
        return self.start_butterfly()

    def stop_butterfly(self, restore=True, now=None):
        if not self.face.butterfly.active:
            return
        state = self.butterfly_resume_state or 'normal'
        self.face.butterfly.stop()
        self.butterfly_resume_state = None
        automatic = self.butterfly_automatic
        if not automatic:
            self.face.note_interaction()
        self.face.bubble_text = ''
        if now is not None and not automatic:
            self.face.last_interaction = now
        if restore:
            self._show_reaction(state)

    def update_companion(self, now=None):
        now = time.monotonic() if now is None else now
        if self.face.split_play.active:
            return
        if self.face.play.mode is not None:
            state, text, finished = self.face.play.stage(now)
            self._show_reaction(state)
            if text != self.face.bubble_text or now >= self.face.bubble_until:
                self.face.say(text, 4.)
            if finished:
                mode = self.face.play.mode
                resume = self.face.play.resume
                self.face.play.stop()
                self._show_reaction(resume)
                if mode == 'hide' and self.face.last_motion_sample is not None:
                    self.face.return_sample = self.face.last_motion_sample.copy()
                    self.face.return_started = now
                elif mode != 'hide':
                    self.face.note_interaction()
            return
        if self.tooth_reaction_started is not None:
            elapsed = now-self.tooth_reaction_started
            stage = (4 if elapsed >= 14. else 3 if elapsed >= 8.4 else
                     2 if elapsed >= 3.4 else 1 if elapsed >= 1.6 else 0)
            if stage != self.tooth_reaction_stage:
                self.tooth_reaction_stage = stage
                state, text = {1: ('astonished', 'えっ！？ 前歯が！'),
                               2: ('crying', 'うわーん、抜けちゃった…'),
                               3: ('troubled', 'しょんぼり…'),
                               4: ('normal', 'もう大丈夫なのだ')}[stage]
                self._show_reaction(state)
                self.face.say(text, 3.5)
                self.face.last_interaction = now
                if stage == 2 and not self.speech_active:
                    self.voice.try_play('crying')
                if stage == 4:
                    self.tooth_reaction_started = None
            return
        if self.face.butterfly.active:
            elapsed = self.face.butterfly.elapsed(now)
            if elapsed >= Butterfly.DURATION:
                self.stop_butterfly(now=now)
            else:
                state = self.face.butterfly.expression(elapsed)
                if state != self.current_state:
                    self._show_reaction(state)
                    if state == 'curious' and elapsed >= Butterfly.LAND_START:
                        self.face.say('おはなに、とまった？', 2.5)
                    elif state == 'sneezing':
                        self.face.say('はっ… はっ…', .7)
                    elif state == 'relieved':
                        self.face.say('またあそぼうね', 3.)
                        self.face.hands.trigger(('left', 'right'))
                if Butterfly.SNEEZE <= elapsed < Butterfly.SNEEZE+.35:
                    self.face.say('はっくしょん！', .9)
            return
        idle = now-self.face.last_interaction
        if idle < 12.:
            if self.idle_origin_state is not None:
                state = self.idle_origin_state
                self.idle_origin_state = None
                self.idle_phase = None
                self._show_reaction(state)
            return
        if self.speech_active or (self.last_motion_time is not None and
                                  now-self.last_motion_time < 2.):
            return
        if self.face.play.idle_origin != self.face.last_interaction:
            self.face.play.idle_origin = self.face.last_interaction
            self.face.play.idle_seen.clear()
        if idle >= 60. and 'hide' not in self.face.play.idle_seen:
            self.face.play.idle_seen.add('hide')
            self.start_game('hide', now)
            return
        if idle >= 80.:
            self.start_game('sleep', now)
            return
        if idle >= 38. and now-self.last_butterfly_visit >= 60. and self.start_butterfly(now):
            self.butterfly_automatic = True
            return
        phase = int((idle-12.)//8.)
        if phase == self.idle_phase:
            return
        if self.idle_origin_state is None:
            self.idle_origin_state = self.current_state
        self.idle_phase = phase
        state, text = (('normal', 'おさんぽ中'), ('thinking', 'なにしてるの？'),
                       ('radiant', 'こっちだよ！'), ('sleep', 'ひとやすみ…'))[phase % 4]
        self._show_reaction(state)
        self.face.say(text, 3.)
        if phase % 4 == 2:
            self.face.hands.trigger(('left', 'right'))

    def set_speaking(self, active):
        """Keep the mouth moving until the audio player signals completion."""
        active = bool(active)
        if active == self.speech_active:
            return
        if active:
            self.stop_butterfly()
        self.face.talk_active = active
        self.face.talk_level = None
        if active:
            self.speech_resume_state = self.current_state if self.current_state != 'speaking' else 'normal'
            self.speech_active = True
            self.direction_active = False
            # Keep every expression recognizable; the motion renderer opens
            # its own mouth according to the speech envelope.
        else:
            self.speech_active = False
            self.show_state(self.speech_resume_state)

    def set_mouth_level(self, level):
        self.face.talk_level = max(0., min(1., level))
        if not self.speech_active or self.current_state != 'speaking':
            return
        self.movie.setPaused(True)
        # The first 21 frames monotonically open the mouth.
        self.movie.jumpToFrame(round(max(0.0, min(1.0, level)) * 20))

    def _start_movie(self, image_path, finished_callback=None):
        previous = self.movie.currentPixmap().copy() if self.movie is not None else None
        movie = QMovie(str(image_path))
        movie.setCacheMode(QMovie.CacheAll)
        movie.start()
        if not movie.isValid():
            movie.stop()
            raise RuntimeError(f'顔画像を読み込めません: {image_path}')
        if self.movie is not None:
            self.movie.stop()
        self.movie = movie
        self.face.reset_tooth()
        if finished_callback is not None:
            self.movie.finished.connect(finished_callback)
        self.face.previous_pixmap = previous
        self.face.transition_started = time.monotonic()
        self.face.expression_started = self.face.transition_started
        self.face.expression_override = (self.current_state if self.current_state in NATIVE_EXPRESSIONS
                                         and Path(image_path) == self.state_paths[self.current_state] else None)
        self.face.setMovie(self.movie)
        self._scale_movie()
        self.movie.start()
        if Path(image_path).stem == 'bearded_face_licking':
            self.movie.jumpToFrame(0)
            self.movie.setPaused(True)

    def handle_face_touch(self, x, y):
        """Map window coordinates through letterboxing into the source canvas."""
        if self.face.play.mode == 'sleep':
            self.face.play.start('wake', time.monotonic(), self.face.play.resume)
            self._show_reaction('astonished')
            self.face.say('わっ！ おはよう！', 1.5)
            self.face.note_interaction()
            return
        if self.face.play.mode == 'food':
            if self.face.play.food_hit(x, y, self.face.width()/max(1, self.face.height())):
                self.face.say('つかんで口まで持ってきてね', 3.)
            return
        if self.face.play.mode is not None:
            resume = self.face.play.resume
            self.face.play.stop()
            self._show_reaction(resume)
        if self.face.split_play.active:
            self.face.split_play.touch(x, y)
            self.face.note_interaction()
            self.face.say('ぽんっ！', .7)
            return
        target = self.face.face_rect()
        if target.width() <= 0 or target.height() <= 0:
            return
        point = self.face.motion.source_point(
            QPointF(x * self.face.width(), y * self.face.height()), target,
            self.face.last_motion_sample or self.face.motion_sample())
        self.stop_butterfly()
        self.face.note_interaction()
        self.handle_touch(point.x(), point.y())

    def handle_touch(self, x, y):
        """Select an expression without immediately losing it to head tracking."""
        # The tooth remains interactive during the matching cavity voice clip.
        if self.face.drop_front_tooth(x, y):
            self.tooth_reaction_started = time.monotonic()
            self.tooth_reaction_stage = 0
            self.idle_origin_state = None
            self.idle_phase = None
            self.face.say('あっ、前歯が！', 2.)
            self.voice.try_play('tooth_drop')
            return
        now = time.monotonic()
        if self._inside_cheek(x, y, .39) or self._inside_cheek(x, y, .61):
            reaction = self.face.play.tickle(now)
            if reaction is not None:
                self._show_reaction(reaction[0])
                self.face.say(reaction[1])
                self.touch_times.clear()
                return
        if self.speech_active:
            return
        self.direction_active = False
        self.direction_suspended_until = now + 1.0
        self.touch_times = [stamp for stamp in self.touch_times if now - stamp < 1.2]
        self.touch_times.append(now)
        tongue_touched = ((x - 0.50) / 0.075) ** 2 + ((y - 0.69) / 0.07) ** 2 <= 1.0
        if tongue_touched:
            self.show_state('licking')
            self.voice.try_play('licking')
            return
        if len(self.touch_times) >= 3:
            self.show_state('error')
            self.touch_times.clear()
        elif self._inside_eye(x, y, 0.42):
            self.play_wink('left')
        elif self._inside_eye(x, y, 0.58):
            self.play_wink('right')
        elif 0.34 <= x <= 0.66 and 0.15 <= y <= 0.31:
            self.show_state('shy')
            self.face.say('なでなで、うれしいな')
        elif ((x-.50)/.045)**2 + ((y-.50)/.065)**2 <= 1.:
            self.show_state('astonished')
            self.face.say('わっ！')
        elif self._inside_cheek(x, y, 0.39) or self._inside_cheek(x, y, 0.61):
            self.show_state('surprise')
        else:
            # A random touch reaction must never start the grinning hand loop.
            candidates = [i for i, path in enumerate(self.image_paths)
                          if path != self.state_paths.get('smile')]
            extras = [state for state in EXTRA_EXPRESSIONS if state in self.state_paths]
            if extras and random.random() < .35:
                self.show_state(random.choice(extras))
            elif candidates:
                self.show_animation(random.choice(candidates))
        if self.current_state not in EXPRESSION_VOICE_CUES:
            self.voice.try_play()

    def handle_drag(self, delta_x, delta_y):
        if self.current_state == 'moving' or self.speech_active:
            return
        now = time.monotonic()
        if self.face.play.mode != 'bounce':
            sample = self.face.last_motion_sample or self.face.motion_sample()
            self.start_game('bounce', now)
            target = self.face.face_rect()
            self.face.play.x = .5+sample['sway']*target.width()/max(1, self.face.width())
            self.face.play.y = .5+sample['bob']*target.height()/max(1, self.face.height())
        self.face.play.drag(delta_x, delta_y, now-self.face.pointer_time)
        self.face.note_interaction()

    def finish_drag(self):
        if self.face.play.mode == 'bounce':
            self.face.play.release(time.monotonic())
            self.face.say('ぽよーん！', 2.)

    def start_game(self, mode, now=None):
        if self.current_state == 'moving' or self.speech_active or self.tooth_reaction_started is not None:
            return False
        now = time.monotonic() if now is None else now
        self.face.split_play.stop()
        self.stop_butterfly()
        resume = self.idle_origin_state or (self.face.play.resume if self.face.play.mode else self.current_state)
        self.idle_origin_state = self.idle_phase = None
        self.direction_active = False
        self.face.return_sample = None
        self.face.play.start(mode, now, resume)
        state, text, _ = self.face.play.stage(now)
        self._show_reaction(state)
        self.face.say(text, 4.)
        return True

    def finish_food(self):
        if self.face.play.mode != 'food':
            return
        x, y = self.face.play.food
        point = self.face.motion.source_point(QPointF(x*self.face.width(), y*self.face.height()),
                                             self.face.face_rect(), self.face.motion_sample())
        if .40 <= point.x() <= .60 and .57 <= point.y() <= .78:
            self.face.play.start('eating', time.monotonic(), self.face.play.resume)
            self._show_reaction('normal')
            self.face.say('もぐもぐ…', 3.)
        else:
            self.face.play.food = (.79, .72)
            self.face.say('口まで持ってきてね', 3.)

    def _return_drag_to_center(self):
        frame_count = self.movie.frameCount()
        center = frame_count // 2
        difference = center - self.drag_frame
        if difference == 0:
            self.return_timer.stop()
            self.drag_frame = None
            self.show_state(self.drag_resume_state)
            return
        step = max(1, abs(difference) // 4)
        self.drag_frame += step if difference > 0 else -step
        self.movie.jumpToFrame(self.drag_frame)

    @staticmethod
    def _inside_eye(x, y, center_x):
        return ((x - center_x) / 0.065) ** 2 + ((y - 0.39) / 0.065) ** 2 <= 1.0

    @staticmethod
    def _inside_cheek(x, y, center_x):
        return ((x - center_x) / 0.075) ** 2 + ((y - 0.52) / 0.07) ** 2 <= 1.0

    def play_wink(self, side):
        """Play a one-shot wink and then return to the selected loop."""
        if self.speech_active:
            return
        resume_state = self.current_state
        self._start_movie(
            self.wink_paths[side],
            lambda: self.show_state(resume_state),
        )

    def _scale_movie(self):
        if self.movie is None:
            return
        # Decode at original resolution. Scaling QMovie on each resize/state
        # can change QLabel's size hint and feed back into window geometry.
        self.movie.setScaledSize(QSize())
        self.face.update()

    def closeEvent(self, event):
        self.companion_timer.stop()
        self.face.split_play.stop()
        self.face.animation_timer.stop()
        self.face.tooth_timer.stop()
        self.voice.close()
        super().closeEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale_movie()

    def play_birthday_greeting(self):
        """Play the birthday clip only when its dedicated key is pressed."""
        if self.speech_active or not self.voice.try_play('birthday'):
            return
        self.face.note_interaction()
        self.face.play.stop()
        self.face.split_play.stop()
        self.stop_butterfly(restore=False)
        self.tooth_reaction_started = None
        self.idle_origin_state = self.idle_phase = None
        self._show_reaction('radiant')
        self.face.say('お誕生日おめでとう！', 3.)

    def motion_received(self, moving):
        if moving:
            self.face.note_interaction()
            self.last_motion_time = time.monotonic()
            self.show_state('moving')
            if not self.isVisible():
                self.show() if self.transparent_background else self.showFullScreen()
                self.raise_()
                self.activateWindow()

    def check_stale_command(self):
        if self.last_motion_time is None:
            return
        if time.monotonic() - self.last_motion_time >= self.stop_delay:
            self.last_motion_time = None
            if self.current_state == 'moving' or (self.speech_active and self.speech_resume_state == 'moving'):
                self.show_state('normal')

    def keyPressEvent(self, event):
        games = {Qt.Key_F: 'food', Qt.Key_C: 'hide', Qt.Key_Z: 'sleep'}
        if event.key() in games:
            if not event.isAutoRepeat():
                self.face.note_interaction()
                self.start_game(games[event.key()])
            event.accept()
            return
        if event.key() == Qt.Key_X:
            if not event.isAutoRepeat():
                if self.face.split_play.active:
                    self.face.split_play.merge()
                else:
                    self.start_split_play()
            event.accept()
            return
        if event.key() == Qt.Key_B:
            self.face.split_play.stop()
            self.face.play.stop()
            if not event.isAutoRepeat():
                self.recall_butterfly()
            event.accept()
            return
        if event.key() == Qt.Key_7:
            if not event.isAutoRepeat():
                self.play_birthday_greeting()
            event.accept()
            return
        self.face.split_play.stop()
        self.face.play.stop()
        self.stop_butterfly()
        self.face.note_interaction()
        expressions = {Qt.Key_1: 'crying', Qt.Key_2: 'shy', Qt.Key_3: 'astonished',
                       Qt.Key_4: 'radiant', Qt.Key_5: 'curious', Qt.Key_6: 'excited',
                       Qt.Key_0: 'pout', Qt.Key_8: 'relieved', Qt.Key_9: 'dizzy'}
        if event.key() in expressions:
            self.show_state(expressions[event.key()])
            return
        if event.key() == Qt.Key_A:
            self.show_state('laughing')
            return
        if event.key() == Qt.Key_G:
            self.show_state('smile')
            return
        if event.key() == Qt.Key_H:
            self.face.hands.trigger(('left', 'right'))
            return
        if event.key() in (Qt.Key_N, Qt.Key_P):
            self.show_state('nodding' if event.key() == Qt.Key_N else 'playful')
            return
        if event.key() == Qt.Key_S:
            self.set_speaking(not self.speech_active)
            return
        if event.key() == Qt.Key_L:
            self.direction_suspended_until = time.monotonic() + 1.0
            self.show_state('licking')
            return
        if event.key() == Qt.Key_Escape:
            self.hide()
            return
        if event.key() == Qt.Key_Q:
            QApplication.quit()
            return
        super().keyPressEvent(event)


class VelocityMonitor:
    def __init__(self, window, topic, state_topic='/face_state', speaking_topic='/face_speaking'):
        self.window = window
        self.node = rclpy.create_node('robot_face_display')
        self.subscription = self.node.create_subscription(Twist, topic, self.callback, 10)
        self.state_subscription = self.node.create_subscription(
            String, state_topic, self.state_callback, 10)
        self.speaking_subscription = self.node.create_subscription(
            Bool, speaking_topic, self.speaking_callback, 10)

    def callback(self, message):
        self.window.motion_received(is_moving(message))

    def state_callback(self, message):
        self.window.face.note_interaction()
        self.window.show_state(message.data)

    def speaking_callback(self, message):
        self.window.set_speaking(message.data)

    def spin_once(self):
        if not rclpy.ok():
            QApplication.quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except KeyboardInterrupt:
            QApplication.quit()

    def close(self):
        self.node.destroy_node()


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--image', nargs='+', default=[str(path) for path in DEFAULT_FACES],
        help='順番に切り替える顔画像（GIF/PNG、複数指定可）')
    parser.add_argument('--left-wink', default=str(DEFAULT_WINKS['left']))
    parser.add_argument('--right-wink', default=str(DEFAULT_WINKS['right']))
    parser.add_argument('--turn-control', help='Optional custom head-turn GIF')
    parser.add_argument('--transparent', action='store_true', help='背景を透過し、デスクトップ上に顔を表示する')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--state-topic', default='/face_state')
    parser.add_argument('--speaking-topic', default='/face_speaking')
    parser.add_argument('--voice-directory', type=Path, default=ASSET_DIRECTORY / 'voice')
    parser.add_argument('--speech-probability', type=float, default=.3)
    parser.add_argument(
        '--screen', type=int,
        default=int(os.environ.get('ROBOTBASE_FACE_SCREEN', '0')),
        help='表示先モニター番号（0始まり）')
    parser.add_argument('--stop-delay', type=float, default=0.8)
    args = parser.parse_args()
    if not 0 <= args.speech_probability <= 1:
        parser.error('--speech-probability must be between 0 and 1')
    return args


def main():
    args = parse_arguments()
    image_paths = [Path(path).expanduser() for path in args.image]
    missing = [path for path in image_paths if not path.is_file()]
    if missing:
        raise SystemExit(f'顔画像が見つかりません: {missing[0]}')

    rclpy.init(args=None)
    app = QApplication(sys.argv[:1])
    wink_paths = {
        'left': Path(args.left_wink).expanduser(),
        'right': Path(args.right_wink).expanduser(),
    }
    missing_winks = [path for path in wink_paths.values() if not path.is_file()]
    if missing_winks:
        raise SystemExit(f'ウインク画像が見つかりません: {missing_winks[0]}')
    turn_control_path = Path(args.turn_control).expanduser() if args.turn_control else None
    if turn_control_path is not None and not turn_control_path.is_file():
        raise SystemExit(f'横向き画像が見つかりません: {turn_control_path}')

    missing_states = [path for path in DEFAULT_STATES.values() if not path.is_file()]
    if missing_states:
        raise SystemExit(f'状態画像が見つかりません: {missing_states[0]}')

    window = FaceWindow(
        image_paths, args.screen, args.stop_delay, wink_paths,
        turn_control_path=turn_control_path,
        voice_directory=args.voice_directory, speech_probability=args.speech_probability,
        transparent_background=args.transparent)
    window.show() if args.transparent else window.showFullScreen()
    monitor = VelocityMonitor(window, args.topic, args.state_topic, args.speaking_topic)

    ros_timer = QTimer()
    ros_timer.timeout.connect(monitor.spin_once)
    ros_timer.start(20)
    stale_timer = QTimer()
    stale_timer.timeout.connect(window.check_stale_command)
    stale_timer.start(100)

    def stop_runtime():
        ros_timer.stop()
        stale_timer.stop()
        window.voice.close()
        monitor.close()

    app.aboutToQuit.connect(stop_runtime)
    try:
        exit_code = app.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
