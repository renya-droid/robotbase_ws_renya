"""Continuous expression-aware animation using the existing face artwork."""
import math
import time
from collections import deque
from mouth_art import MouthArt
from expression_art import NATIVE_EXPRESSIONS, draw_expression_eyes, draw_eyebrow

from PyQt5.QtCore import QPointF, QRect, QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen, QPixmap, QRegion, QTransform


def interpolate(value, points):
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if value <= x1:
            return y0 + (value - x0) * (y1 - y0) / (x1 - x0)
    return value


class FaceMotion:
    # Source-image eye and mouth bands. Deformation tapers into adjacent rows,
    # so there are no rectangular cutouts around facial features.
    BANDS = {
        'laughing': (.315, .380, .575, .745),
        'radiant': (.315, .380, .575, .735),
        'crying': (.320, .445, .650, .715),
        'shy': (.335, .440, .650, .705),
        'astonished': (.245, .415, .570, .710),
    }

    def __init__(self):
        self.mouth_art = MouthArt()
        self.started = time.monotonic()
        self.eye_cache = {}
        self.brow_cache = {}
        self.nose_cache = {}
        self.mouth_cache = {}

    @staticmethod
    def blink_amount(t):
        # Unequal intervals and an occasional double blink. Closing is faster
        # than reopening; sampling never changes the schedule or uses RNG.
        phase = t % 32.
        pulses = []
        for center in (3.1, 7.9, 10.5, 16.2, 16.48, 21.8, 25.4, 29.6):
            delta = phase-center
            width = .065 if delta < 0. else .125
            pulses.append(math.exp(-(delta/width)**2))
        return max(pulses)

    @staticmethod
    def smoothstep(value):
        value = max(0., min(1., value))
        return value*value*(3.-2.*value)

    def sample(self, expression, gaze_x=0., gaze_y=0., speaking=False,
               mouth_level=None, elapsed=None, idle_seconds=0.,
               head_gaze=None, expression_age=None):
        t = time.monotonic() - self.started if elapsed is None else elapsed
        blink = self.blink_amount(t)
        eye = 1.0 - .94 * blink
        mouth = .04 + .045 * (.5 + .5 * math.sin(t * 1.3))
        # Quiet breathing and posture adjustments have different rhythms;
        # avoid making every part of the face bob on the same metronome.
        breath = math.sin(t*1.55 + .16*math.sin(t*.31))
        stillness = .65 + .35*(.5+.5*math.sin(t*.43+.7))
        sway = stillness*(.0017*math.sin(t*.77)+.0006*math.sin(t*1.91))
        bob = .0017*breath + stillness*.0008*math.sin(t*.53)
        tilt = stillness*(.3*math.sin(t*.61)+.12*math.sin(t*1.37))
        rhythm = t + .13*math.sin(t*.71)
        age = max(0., expression_age) if expression_age is not None else None
        if expression in ('happy', 'laughing', 'radiant', 'smile', 'playful', 'success'):
            mouth = .58 + .32 * math.sin(rhythm * 2.6)
            bob += .003 * math.sin(rhythm * 2.6)
            # Already-smiling closed eyes should bob rather than vanish.
            eye = .96 + .04 * math.sin(t * 5.5)
        elif expression == 'curious':
            mouth = .16+.08*math.sin(t*1.7)
            tilt += 3.0
            eye *= 1.08
        elif expression == 'excited':
            mouth = .65+.20*math.sin(rhythm*3.5)
            bob += .004*math.sin(rhythm*3.5)
            eye *= 1.12
        elif expression == 'pout':
            mouth = .035+.025*(.5+.5*math.sin(t*1.8))
            tilt -= 1.8
            eye *= .72
        elif expression == 'relieved':
            mouth = .08+.05*(.5+.5*math.sin(t*1.2))
            bob += .003*(.5+.5*math.sin(t*.9))
        elif expression == 'dizzy':
            mouth = .3+.12*math.sin(t*2.3)
            tilt += 2.8*math.sin(t*1.9)
        elif expression == 'sneezing':
            sneeze_age = .7 if age is None else age
            intake = self.smoothstep(sneeze_age/.65)
            burst = math.exp(-((sneeze_age-.70)/.115)**2)
            recovery = 1.-self.smoothstep((sneeze_age-.90)/.6)
            mouth = (.12+.43*intake+.44*burst)*recovery
            eye = max(.06, 1.-.70*intake-.60*burst)
            bob += -.008*intake*recovery+.025*burst
            tilt += -2.*intake*recovery+5.*burst
        elif expression == 'licking':
            mouth = .66 + .10 * math.sin(t * 4.4)
        elif expression == 'crying':
            mouth = .52 + .30 * math.sin(t * 3.2)
            sway += .0015 * math.sin(t * 13.)
            bob += .002 * math.sin(t * 3.2)
            tilt += .25 * math.sin(t * 9.)
        elif expression == 'shy':
            mouth = .06 + .07 * (.5 + .5 * math.sin(t * 1.8))
            tilt -= 1.8
            bob += .004
            eye *= .88 + .04 * math.sin(t * 2.)
        elif expression in ('astonished', 'surprise'):
            eye = 1.04 - .65 * blink
            mouth = .65 + .12 * math.sin(t * 1.8)
            if age is not None:
                # A quick intake of breath, then settle into the expression.
                gasp = math.exp(-((age-.20)/.19)**2)
                mouth = mouth*(.45+.55*self.smoothstep(age/.3))+.22*gasp
                eye += .12*gasp
        elif expression == 'sleep':
            eye = .40 + .08 * math.sin(t * 1.3)
            mouth = .04 + .03 * math.sin(t * 1.3)
        elif expression in ('error', 'troubled'):
            eye *= .83
            tilt += .8 * math.sin(t * 3.)
            mouth = .10 + .08 * (.5 + .5 * math.sin(t * 2.4))
        if expression == 'crying' and age is not None:
            build = self.smoothstep((age-.15)/.85)
            mouth *= .35+.65*build
            tilt *= .65+.35*build
        # Natural rests close the lips between smiles and expressions.
        closure = math.exp(-(((t % 6.2) - 5.1) / .42) ** 2)
        if expression != 'sneezing':
            mouth *= 1. - .99 * closure
        if speaking:
            level = max(0., math.sin(t * 9.) * math.sin(t * 2.7)) if mouth_level is None else mouth_level
            mouth = .025 + .975 * max(0., min(1., level)) ** .7
        mouth = max(.025, min(1., mouth))
        # Existing one-shot winks and mouth-frame playback stay recognizable.
        if expression.startswith('wink'):
            eye = 1.0
        eye_top, eye_bottom, mouth_top, mouth_bottom = self.BANDS.get(
            expression, (.322, .425, .642, .745))
        points = [(0., 0.)]
        for top, bottom, factor, padding in (
                (mouth_top, mouth_bottom, mouth, .045),):
            center, half = (top + bottom) / 2, (bottom - top) / 2
            points.extend(((top - padding, top - padding),
                           (top, center - half * factor),
                           (bottom, center + half * factor),
                           (bottom + padding, bottom + padding)))
        points.append((1., 1.))
        idle_blend = max(0., min(1., (idle_seconds - 12.) / 5.))
        idle_blend = idle_blend ** 2 * (3. - 2. * idle_blend)
        travel_time = max(0., idle_seconds - 12.)
        # Stroll, pause to look around, wave, then rest like a desktop mascot.
        cycle, phase = divmod(travel_time, 32.)
        walk_time = cycle*26. + min(phase, 20.) + max(0., phase-26.)
        sway += idle_blend * .30 * math.sin(walk_time * .115)
        bob += idle_blend * .18 * math.sin(walk_time * .083)
        if phase < 18.:
            bob += idle_blend * .004 * abs(math.sin(travel_time*3.))
            tilt += idle_blend * 1.1 * math.sin(travel_time*1.6)
        head_x, head_y = (gaze_x, gaze_y) if head_gaze is None else head_gaze
        # Small held glances with short transitions rather than constant eye drift.
        glances = (0., .55, .12, -.45, -.14, .32, 0.)
        look_phase = t/2.3
        look_index = int(look_phase) % len(glances)
        amount = self.smoothstep((look_phase % 1.)/.065)
        glance = glances[(look_index-1) % len(glances)]*(1.-amount)+glances[look_index]*amount
        return {'points': points, 'sway': sway + .003 * head_x,
                'bob': bob + .002 * head_y, 'tilt': tilt,
                'scale': (1. - .45 * idle_blend) * (1. + .0018 * breath),
                'idle_blend': idle_blend, 'mouth_open': mouth,
                'mouth_top': mouth_top, 'mouth_bottom': mouth_bottom,
                'eye_factor': eye, 'eye_top': eye_top, 'eye_bottom': eye_bottom,
                'expression': expression, 'elapsed': t, 'gaze': (gaze_x, gaze_y),
                'eye_center': (eye_top + eye_bottom) / 2,
                'eye_shift': .003*gaze_x + .0008*glance}

    @staticmethod
    def transform(target, sample):
        transform = QTransform()
        transform.translate(target.center().x() + target.width() * sample['sway'],
                            target.center().y() + target.height() * sample['bob'])
        transform.rotate(sample['tilt'])
        transform.scale(sample['scale'], sample['scale'])
        transform.translate(-target.center().x(), -target.center().y())
        return transform

    @staticmethod
    def eye_shift(y, sample):
        return sample['eye_shift'] * math.exp(-((y - sample['eye_center']) / .065) ** 4)

    def draw(self, painter, target, pixmap, sample, eye_key=None):
        painter.save()
        painter.setTransform(self.transform(target, sample), True)
        if sample.get('transparent_background'):
            painter.setClipPath(self.face_outline(target), Qt.IntersectClip)
        # Keep nose, moustache and beard stable; only the isolated mouth moves.
        painter.drawPixmap(QRectF(target), pixmap, QRectF(pixmap.rect()))
        self.draw_mouth(painter, target, pixmap, sample, eye_key)
        self.draw_brows(painter, target, pixmap, sample, eye_key)
        if sample['expression'] in NATIVE_EXPRESSIONS or sample['expression'] == 'error':
            draw_expression_eyes(painter, target, sample)
            if sample['expression'] == 'error':
                self.restore_nose(painter, target, pixmap, eye_key)
        elif not sample['expression'].startswith('wink'):
            self.draw_eyes(painter, target, pixmap, sample, eye_key)
        self.draw_contour(painter, target)
        painter.restore()

    def restore_nose(self, painter, target, pixmap, key):
        key = key if key is not None else pixmap.cacheKey()
        if key not in self.nose_cache:
            image, mask = pixmap.toImage(), QRegion()
            for y in range(round(pixmap.height()*.375), round(pixmap.height()*.56)):
                for x in range(round(pixmap.width()*.45), round(pixmap.width()*.55)):
                    c = image.pixelColor(x, y)
                    if c.red() > 70 and c.green() > 35 and c.red()-c.blue() > 35:
                        mask |= QRegion(x-1, y-1, 3, 3)
            if len(self.nose_cache) >= 32:
                self.nose_cache.pop(next(iter(self.nose_cache)))
            self.nose_cache[key] = mask
        painter.save()
        painter.translate(target.x(), target.y())
        painter.scale(target.width()/pixmap.width(), target.height()/pixmap.height())
        painter.setClipRegion(self.nose_cache[key])
        painter.drawPixmap(0, 0, pixmap)
        painter.restore()

    def draw_brows(self, painter, target, pixmap, sample, key):
        """Remove source eyebrow hair, then use one silhouette for every state."""
        key = (key, sample['eye_top'])
        if key not in self.brow_cache:
            image, candidates = pixmap.toImage(), set()
            eye_top = sample['eye_top']
            for y in range(round(pixmap.height()*.07), round(pixmap.height()*(eye_top+.025))):
                for x in range(round(pixmap.width()*.30), round(pixmap.width()*.70)):
                    c = image.pixelColor(x, y)
                    values = (c.red(), c.green(), c.blue())
                    if 22 < max(values) < 170 and max(values)-min(values) < 45:
                        candidates.add((x, y))
            mask = QRegion()
            while candidates:
                seed = candidates.pop()
                component, queue = [seed], deque([seed])
                while queue:
                    x, y = queue.popleft()
                    for point in ((x-1, y), (x+1, y), (x, y-1), (x, y+1),
                                  (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)):
                        if point in candidates:
                            candidates.remove(point); component.append(point); queue.append(point)
                if (len(component) < 25 or min(y for x, y in component) >= (eye_top-.025)*pixmap.height()
                        or max(x for x, y in component)-min(x for x, y in component) < pixmap.width()*.018):
                    continue
                left = min(x for x, y in component)-2
                top = min(y for x, y in component)-2
                right = max(x for x, y in component)+2
                bottom = max(y for x, y in component)+2
                mask |= QRegion(left, top, right-left+1, bottom-top+1)
            if len(self.brow_cache) >= 32:
                self.brow_cache.pop(next(iter(self.brow_cache)))
            self.brow_cache[key] = mask
        painter.save()
        painter.translate(target.x(), target.y())
        painter.scale(target.width()/pixmap.width(), target.height()/pixmap.height())
        painter.setClipRegion(self.brow_cache[key])
        painter.fillRect(pixmap.rect(), Qt.black)
        painter.restore()
        painter.save()
        painter.translate(target.x(), target.y()); painter.scale(target.width(), target.height())
        painter.setRenderHint(QPainter.Antialiasing)
        for side, cx in enumerate((.417, .583)):
            draw_eyebrow(painter, sample['expression'], side, cx, sample['eye_top']-.052)
        painter.restore()

    @staticmethod
    def face_outline(target):
        outline = QPainterPath(QPointF(.5, .065))
        outline.cubicTo(.34, .065, .285, .22, .294, .42)
        outline.cubicTo(.255, .405, .25, .545, .298, .565)
        outline.cubicTo(.31, .79, .385, .955, .5, .965)
        outline.cubicTo(.615, .955, .69, .79, .702, .565)
        outline.cubicTo(.75, .545, .745, .405, .706, .42)
        outline.cubicTo(.715, .22, .66, .065, .5, .065)
        transform = QTransform()
        transform.translate(target.x(), target.y())
        transform.scale(target.width(), target.height())
        return transform.map(outline)

    @staticmethod
    def draw_contour(painter, target):
        painter.save()
        painter.translate(target.x(), target.y())
        painter.scale(target.width(), target.height())
        outline = FaceMotion.face_outline(QRectF(0., 0., 1., 1.))
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor('#bc8e72'), max(.0022, .85/max(1., target.height())), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(outline)
        painter.restore()

    def draw_mouth(self, painter, target, pixmap, sample, key):
        # Cache the static cleanup mask; lips and teeth are drawn independently.
        key = (key, pixmap.cacheKey())
        if key not in self.mouth_cache:
            bounds = QRect(round(.375 * pixmap.width()), round(sample['mouth_top'] * pixmap.height()),
                           round(.25 * pixmap.width()),
                           round((sample['mouth_bottom']-sample['mouth_top']) * pixmap.height()))
            image, mask = pixmap.toImage(), QRegion()
            candidates, seeds = set(), []
            for y in range(bounds.top(), bounds.bottom() + 1):
                for x in range(bounds.left(), bounds.right() + 1):
                    c = image.pixelColor(x, y)
                    selected = ((c.red() > 3 and c.red() > c.green() * 1.32
                                 and c.red() > c.blue() * 1.15)
                                or (c.red() > 180 and c.green() > 150 and c.blue() > 90))
                    if selected:
                        candidates.add((x, y))
                        if c.red() > 90:
                            seeds.append((x, y))
            pixels = set()
            if seeds:
                center = bounds.center()
                seed = min(seeds, key=lambda xy: (xy[0]-center.x())**2 + (xy[1]-center.y())**2)
                queue = deque([seed])
                pixels.add(seed)
                while queue:
                    x, y = queue.popleft()
                    for point in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
                        if point in candidates and point not in pixels:
                            pixels.add(point)
                            queue.append(point)
            rows = {}
            for x, y in pixels:
                left, right = rows.get(y, (x, x))
                rows[y] = min(left, x), max(right, x)
            for y, (left, right) in rows.items():
                mask |= QRegion(left, y, right-left+1, 1)
            for x, y in candidates:
                if .39*pixmap.width() < x < .61*pixmap.width():
                    mask |= QRegion(x, y, 1, 1)
            actual = mask.boundingRect()
            # The generated artwork has faint red speckles around the mouth.
            # Clear its whole footprint while preserving the beard's own color.
            protected = QRegion()
            hair_pixels = set()
            for y in range(bounds.top(), bounds.bottom()+1):
                start = None
                for x in range(bounds.left(), bounds.right()+2):
                    keep = False
                    if x <= bounds.right():
                        c = image.pixelColor(x, y)
                        # Protect both dark outlines and lighter brown hair,
                        # rather than matching only one sampled beard shade.
                        keep = (12 < max(c.red(), c.green(), c.blue()) < 145
                                and .95*c.green() <= c.red() <= 1.33*c.green()
                                and .95*c.blue() <= c.green() <= 1.35*c.blue())
                    if keep:
                        hair_pixels.add((x, y))
                    if keep and start is None:
                        start = x
                    elif not keep and start is not None:
                        protected |= QRegion(start, y, x-start, 1)
                        start = None
            # Only connected hair belongs in the foreground. Isolated dark
            # compression speckles inside the old mouth are not whiskers.
            while hair_pixels:
                seed = hair_pixels.pop()
                component, queue = [seed], deque([seed])
                while queue:
                    x, y = queue.popleft()
                    for point in ((x-1, y), (x+1, y), (x, y-1), (x, y+1),
                                  (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)):
                        if point in hair_pixels:
                            hair_pixels.remove(point)
                            component.append(point)
                            queue.append(point)
                if len(component) < 24:
                    for x, y in component:
                        protected -= QRegion(x, y, 1, 1)
            # Do not expand this mask into adjacent old white teeth.
            fixed_beard = protected
            mask = QRegion(actual) - fixed_beard
            # Include faint antialiased mouth edges so closing the lips never
            # leaves a dotted ghost of the open mouth behind.
            original = mask
            for offset in range(-3, 4):
                mask |= original.translated(offset, 0)
                mask |= original.translated(0, offset)
            # Dilation must not pull moustache pixels back into the mouth.
            mask -= fixed_beard
            beard_layer = QPixmap(bounds.size())
            beard_layer.fill(Qt.transparent)
            draw = QPainter(beard_layer)
            draw.translate(-bounds.x(), -bounds.y())
            draw.setClipRegion(fixed_beard)
            draw.drawPixmap(0, 0, pixmap)
            draw.end()
            if len(self.mouth_cache) >= 24:
                self.mouth_cache.pop(next(iter(self.mouth_cache)))
            self.mouth_cache[key] = bounds, actual, mask, beard_layer
        bounds, actual, mask, beard_layer = self.mouth_cache[key]
        if actual.isEmpty():
            return
        painter.save()
        painter.translate(target.x(), target.y())
        painter.scale(target.width()/pixmap.width(), target.height()/pixmap.height())
        painter.setClipRegion(mask)
        painter.fillRect(bounds, Qt.black)
        painter.restore()
        self.mouth_art.draw(painter, target, sample)
        # Keep the original hair in front of the moving lips at its original
        # position, even when the mouth grows into the moustache's footprint.
        painter.drawPixmap(QRectF(target.x() + bounds.x()/pixmap.width()*target.width(),
                                  target.y() + bounds.y()/pixmap.height()*target.height(),
                                  bounds.width()/pixmap.width()*target.width(),
                                  bounds.height()/pixmap.height()*target.height()),
                           beard_layer, QRectF(beard_layer.rect()))
        self.mouth_art.draw_tongue(painter, target, sample)

    def draw_eyes(self, painter, target, pixmap, sample, key):
        key = key if key is not None else pixmap.cacheKey()
        if key not in self.eye_cache:
            image = pixmap.toImage()
            eyes = []
            for center in (.414, .586):
                bounds = QRect(round((center - .057) * pixmap.width()),
                               round(sample['eye_top'] * pixmap.height()),
                               round(.114 * pixmap.width()),
                               round((sample['eye_bottom'] - sample['eye_top']) * pixmap.height()))
                candidates, seeds = set(), []
                for y in range(bounds.top(), bounds.bottom() + 1):
                    for x in range(bounds.left(), bounds.right() + 1):
                        c = image.pixelColor(x, y)
                        high, low = max(c.red(), c.green(), c.blue()), min(c.red(), c.green(), c.blue())
                        if high > 3 and (high - low < 55 or
                                         (high < 180 and c.green() < 150 and c.blue() < 150)):
                            candidates.add((x, y))
                            if c.red() > 180 and c.green() > 180 and c.blue() > 170:
                                seeds.append((x, y))
                # Grow the eye connected to its white sclera; disconnected
                # eyebrows and tear highlights must never move with the eyelid.
                pixels = set()
                if seeds:
                    middle = bounds.center()
                    seed = min(seeds, key=lambda xy: (xy[0]-middle.x())**2 + (xy[1]-middle.y())**2)
                    queue = deque([seed])
                    pixels.add(seed)
                    while queue:
                        x, y = queue.popleft()
                        for point in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
                            if point in candidates and point not in pixels:
                                pixels.add(point)
                                queue.append(point)
                mask = QRegion()
                for y in range(bounds.top(), bounds.bottom() + 1):
                    start = None
                    for x in range(bounds.left(), bounds.right() + 2):
                        selected = (x, y) in pixels
                        if selected and start is None:
                            start = x
                        elif not selected and start is not None:
                            mask += QRegion(start, y, x-start, 1)
                            start = None
                original = mask
                for offset in range(-4, 5):
                    mask |= original.translated(offset, 0)
                    mask |= original.translated(0, offset)
                bounds = bounds.adjusted(-4, -4, 4, 4)
                layer = QPixmap(bounds.size())
                layer.fill(Qt.transparent)
                draw = QPainter(layer)
                draw.translate(-bounds.x(), -bounds.y())
                draw.setClipRegion(mask)
                draw.drawPixmap(0, 0, pixmap)
                draw.end()
                eyes.append((bounds, mask, layer))
            if len(self.eye_cache) >= 24:
                self.eye_cache.pop(next(iter(self.eye_cache)))
            self.eye_cache[key] = eyes
        # Masks isolate eyes from the nose, tears and surrounding beard.
        for bounds, mask, layer in self.eye_cache[key]:
            painter.save()
            painter.translate(target.x(), target.y())
            painter.scale(target.width()/pixmap.width(), target.height()/pixmap.height())
            painter.setClipRegion(mask)
            painter.fillRect(bounds, Qt.black)
            painter.restore()
            rect = QRectF(target.x() + bounds.x()/pixmap.width()*target.width(),
                          target.y() + bounds.y()/pixmap.height()*target.height(),
                          bounds.width()/pixmap.width()*target.width(),
                          bounds.height()/pixmap.height()*target.height())
            rect.translate(sample['eye_shift'] * target.width(), 0.)
            center = rect.center().y()
            rect.setHeight(rect.height() * sample['eye_factor'])
            rect.moveCenter(QPointF(rect.center().x(), center))
            painter.drawPixmap(rect, layer, QRectF(layer.rect()))

    @classmethod
    def source_point(cls, position, target, sample):
        inverse, valid = cls.transform(target, sample).inverted()
        if not valid or target.width() <= 0 or target.height() <= 0:
            return QPointF(-1., -1.)
        local = inverse.map(position)
        y = interpolate((local.y() - target.y()) / target.height(),
                        [(b, a) for a, b in sample['points']])
        x = (local.x() - target.x()) / target.width() - cls.eye_shift(y, sample)
        return QPointF(x, y)
