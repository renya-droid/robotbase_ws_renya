"""Independent, bounded, smoothly animated transparent hand overlays."""
import math
import random
import time
from PyQt5.QtCore import QRectF, QSize, Qt
from pathlib import Path
from PyQt5.QtGui import QPainter, QPixmap, QRegion, QTransform


class HandGesture:
    duration = 2.6

    def __init__(self, image_path, probability=.3, cooldown=3.0):
        image = QPixmap(str(image_path))
        if not image.isNull():
            bounds = QRegion(image.mask()).boundingRect()
            image = image.copy(bounds)
        self.sprites = {'left': image,
                        'right': QPixmap.fromImage(image.toImage().mirrored(True, False))}
        self.grip_frames = {'left': [], 'right': []}
        sheet = QPixmap(str(Path(image_path).with_name('hand_grip_sheet.png')))
        if not sheet.isNull():
            cells = [sheet.copy(round(i*sheet.width()/4), 0,
                     round((i+1)*sheet.width()/4)-round(i*sheet.width()/4), sheet.height())
                     for i in range(4)]
            common = QRegion(cells[0].mask()).boundingRect()
            for cell in cells[1:]:
                common = common.united(QRegion(cell.mask()).boundingRect())
            frame_size = common.size().scaled(QSize(512, 512), Qt.KeepAspectRatio)
            self.grip_frames['left'] = [cell.copy(common).scaled(frame_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation) for cell in cells]
            self.grip_frames['right'] = [QPixmap.fromImage(cell.toImage().mirrored(True, False))
                                         for cell in self.grip_frames['left']]
        self.grip_blends = {}
        self.grip_started_at = None
        self.probability = probability
        self.cooldown = cooldown
        self.started_at = None
        self.last_finished = -float('inf')
        self.sides = ()

    def set_gripping(self, active, now=None):
        if active and self.grip_frames['left']:
            if self.grip_started_at is None:
                self.started_at = None
                self.grip_started_at = time.monotonic() if now is None else now
                self.sides = ('left', 'right')
        elif self.grip_started_at is not None:
            self.grip_started_at = None
            self.started_at = None

    def grip_position(self, side, now=None):
        now = time.monotonic() if now is None else now
        elapsed = max(0.0, now - self.grip_started_at)
        phase = 0 if side == 'left' else .18
        # Three poses of curl and reopen, with slightly staggered hands.
        return 3 * (1 - math.cos(2 * math.pi * (elapsed * .9 + phase))) / 2

    def trigger(self, sides=None, now=None):
        now = time.monotonic() if now is None else now
        if self.sprites['left'].isNull() or self.active(now):
            return False
        if now - self.last_finished < self.cooldown:
            return False
        self.started_at = now
        self.sides = sides or random.choice([('left',), ('right',), ('left', 'right')])
        return True

    def try_trigger(self):
        if random.random() >= self.probability:
            return False
        return self.trigger()

    def active(self, now=None):
        now = time.monotonic() if now is None else now
        if self.grip_started_at is not None:
            return True
        if self.started_at is None:
            return False
        if now - self.started_at >= self.duration:
            self.last_finished = self.started_at + self.duration
            self.started_at = None
            return False
        return True

    def poses(self, target, now=None):
        now = time.monotonic() if now is None else now
        if not self.active(now):
            return []
        gripping = self.grip_started_at is not None
        started = self.grip_started_at if gripping else self.started_at
        elapsed = max(0.0, now - started)
        ramp = max(0.0, min(1.0, elapsed / .4)) if gripping else max(0.0, min(1.0, elapsed / .4, (self.duration - elapsed) / .4))
        ease = ramp * ramp * (3 - 2 * ramp)
        source = self.grip_frames['left'][0] if gripping else self.sprites['left']
        size = source.size().scaled(
            QSize(max(1, round(target.width()*.20)), max(1, round(target.height()*.43))),
            Qt.KeepAspectRatio)
        y = target.y() + round(target.height() * (.43 + .06*(1-ease) + .012*math.sin(elapsed*9)))
        margin = round(target.width() * (.08 + .035*ease if gripping else .025 + .035*ease))
        result = []
        for side in self.sides:
            phase = 0.0 if side == "left" else .65
            sign = 1.0 if side == "left" else -1.0
            angle = sign * (5 if gripping else 20) * math.sin(elapsed * 2 * math.pi * 1.7 + phase) * ease
            sway = target.width() * (.006 if gripping else .012) * math.sin(elapsed * 2 * math.pi * 1.7 + phase) * ease
            x = target.x()+margin if side == 'left' else target.right()-margin-size.width()+1
            local = QRectF(0, 0, size.width(), size.height())
            pivot_x, pivot_y = size.width() * .5, size.height() * .92
            transform = QTransform()
            transform.translate(x + sway + pivot_x, y + pivot_y)
            transform.rotate(angle)
            transform.translate(-pivot_x, -pivot_y)
            bounds = transform.mapRect(local)
            # Constrain the rotated corners, not only the unrotated sprite.
            safe = QRectF(target).adjusted(2, 2, -2, -2)
            dx = max(0.0, safe.left()-bounds.left()) + min(0.0, safe.right()-bounds.right())
            dy = max(0.0, safe.top()-bounds.top()) + min(0.0, safe.bottom()-bounds.bottom())
            transform.setMatrix(transform.m11(), transform.m12(), transform.m13(),
                                transform.m21(), transform.m22(), transform.m23(),
                                transform.dx()+dx, transform.dy()+dy, transform.m33())
            result.append((side, local, ease, transform, transform.mapRect(local)))
        return result

    def layout(self, target, now=None):
        return [(side, bounds.toAlignedRect(), opacity)
                for side, local, opacity, transform, bounds in self.poses(target, now)]

    def paint(self, painter, target):
        painter.save()
        painter.setClipRect(target)
        for side, local, opacity, transform, bounds in self.poses(target):
            painter.save()
            painter.setOpacity(opacity)
            painter.setTransform(transform, True)
            if self.grip_started_at is not None:
                key = (side, round(self.grip_position(side) * 12))
                mixed = self.grip_blends.get(key)
                if mixed is None:
                    position = key[1] / 12
                    lower = min(3, int(position))
                    upper = min(3, lower + 1)
                    fraction = position - lower
                    frames = self.grip_frames[side]
                    mixed = QPixmap(frames[0].size())
                    mixed.fill(Qt.transparent)
                    blend = QPainter(mixed)
                    blend.setOpacity(1-fraction)
                    blend.drawPixmap(0, 0, frames[lower])
                    blend.setCompositionMode(QPainter.CompositionMode_Plus)
                    blend.setOpacity(fraction)
                    blend.drawPixmap(0, 0, frames[upper])
                    blend.end()
                    self.grip_blends[key] = mixed
                painter.drawPixmap(local.toRect(), mixed)
            else:
                painter.drawPixmap(local.toRect(), self.sprites[side])
            painter.restore()
        painter.restore()
