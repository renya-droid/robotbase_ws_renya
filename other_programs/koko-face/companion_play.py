"""Interruptible desktop games; time and pointer coordinates are explicit."""
import math
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QFont, QPainterPath, QPen


class CompanionPlay:
    def __init__(self):
        self.mode = None
        self.started = 0.
        self.resume = 'normal'
        self.held = False
        self.x = self.y = .5
        self.vx = self.vy = 0.
        self.last_step = 0.
        self.food = (.79, .72)
        self.food_held = False
        self.cheek_taps = []
        self.idle_seen = set()
        self.idle_origin = None

    def stop(self):
        self.mode = None
        self.held = self.food_held = False

    def start(self, mode, now, resume='normal'):
        self.stop()
        self.mode, self.started, self.last_step, self.resume = mode, now, now, resume
        self.x = self.y = .5
        self.vx = self.vy = 0.
        self.food = (.79, .72)

    def tickle(self, now):
        self.cheek_taps = [t for t in self.cheek_taps if now-t < 3.]
        self.cheek_taps.append(now)
        if len(self.cheek_taps) >= 6:
            return 'pout', 'くすぐりすぎだよ〜！'
        if len(self.cheek_taps) >= 3:
            return 'laughing', 'あははっ！ くすぐったい！'
        return None

    def drag(self, dx, dy, dt):
        self.held = True
        self.x = max(.16, min(.84, self.x+dx))
        self.y = max(.28, min(.72, self.y+dy))
        self.vx = max(-1.2, min(1.2, dx/max(.016, dt)))
        self.vy = max(-1.2, min(1.2, dy/max(.016, dt)))

    def release(self, now):
        self.held = False
        self.last_step = now
        self.started = now

    def advance(self, now):
        dt = max(0., min(.05, now-self.last_step))
        self.last_step = now
        if self.mode != 'bounce' or self.held:
            return
        self.vy += .6*dt
        self.x += self.vx*dt
        self.y += self.vy*dt
        for axis, velocity, low, high in (('x', 'vx', .16, .84), ('y', 'vy', .28, .72)):
            value = getattr(self, axis)
            if value < low or value > high:
                setattr(self, axis, max(low, min(high, value)))
                setattr(self, velocity, -getattr(self, velocity)*.72)
        self.vx *= math.exp(-dt*.35)

    def stage(self, now):
        age = now-self.started
        if self.mode == 'food':
            return 'curious', 'おやつを口まで持ってきて！', False
        if self.mode == 'eating':
            return ('relieved', 'ごちそうさま！', age >= 4.5) if age >= 3. else ('normal', 'もぐもぐ…', False)
        if self.mode == 'hide':
            return 'playful', 'みーつけた！' if age >= 6. else 'どーこだ？', age >= 9.
        if self.mode == 'sleep':
            return ('sleep', 'すやすや…', False) if age >= 3. else ('normal', 'ふわぁ〜…', False)
        if self.mode == 'wake':
            return 'astonished', 'わっ！ おはよう！', age >= 1.5
        if self.mode == 'bounce':
            return 'excited', 'ぽよーん！', not self.held and age >= 6.
        return None

    def apply_pose(self, sample, now, canvas, target):
        age = now-self.started
        if self.mode == 'bounce':
            settle = 0. if self.held else max(0., min(1., (age-4.)/2.))
            settle = settle*settle*(3.-2.*settle)
            sample.update(scale=.6+.4*settle,
                          sway=(self.x-.5)*canvas.width()/target.width()*(1.-settle),
                          bob=(self.y-.5)*canvas.height()/target.height()*(1.-settle),
                          tilt=self.vx*12.*(1.-settle), idle_blend=0.)
        elif self.mode == 'hide':
            # Travel to the edge, peek twice, then smoothly return to centre.
            enter = min(1., max(0., age/1.5))
            leave = min(1., max(0., (age-6.)/3.))
            enter = enter*enter*(3.-2.*enter)
            leave = leave*leave*(3.-2.*leave)
            amount = enter*(1.-leave)
            sample.update(scale=1.-.58*amount,
                          sway=amount*.49*canvas.width()/target.width(),
                          bob=.03*amount*math.sin(age*3.), tilt=-12.*amount, idle_blend=0.)
        elif self.mode == 'sleep':
            sample.update(sway=0., bob=.003*math.sin(age*1.2), scale=.72,
                          tilt=min(1., age/3.)*9., idle_blend=0.)
            if age < 3.:
                sample['mouth_open'] = .9*math.sin(math.pi*age/3.)
                sample['eye_factor'] = .3
        elif self.mode in ('food', 'eating'):
            sample.update(sway=0., bob=0., scale=.8, idle_blend=0.)
            if self.mode == 'eating' and age < 3.:
                sample['mouth_open'] = .10+.24*(.5+.5*math.sin(age*15.))
                sample['tilt'] += 1.4*math.sin(age*7.5)

    def food_hit(self, x, y, aspect):
        return self.mode == 'food' and ((x-self.food[0])*aspect)**2+(y-self.food[1])**2 < .055**2

    def paint(self, painter, canvas, now, sample):
        if self.mode == 'food':
            x, y = self.food[0]*canvas.width(), self.food[1]*canvas.height()
            radius = canvas.height()*.04
            painter.save(); painter.setRenderHint(painter.Antialiasing)
            painter.setPen(QPen(QColor('#94602c'), max(1., radius*.08)))
            painter.setBrush(QColor('#e5b76b'))
            painter.drawEllipse(QPointF(x, y), radius, radius)
            painter.setPen(Qt.NoPen); painter.setBrush(QColor('#69422b'))
            for dx, dy in ((-.4, -.3), (.3, -.45), (.5, .2), (-.2, .45), (0., 0.)):
                painter.drawEllipse(QPointF(x+dx*radius, y+dy*radius), radius*.12, radius*.12)
            painter.restore()
        if self.mode == 'sleep' and now-self.started >= 3.:
            painter.save(); painter.setRenderHint(painter.Antialiasing)
            font = QFont('Sans Serif', max(12, int(canvas.height()*.04)), QFont.Bold)
            painter.setPen(QPen(QColor('#5777a2'), 1.5)); painter.setBrush(QColor('#e3f1ff'))
            for index in range(3):
                phase = ((now-self.started)*.35+index/3.) % 1.
                painter.setOpacity(1.-phase*.8)
                letters = QPainterPath()
                letters.addText(QPointF(canvas.width()*(.62+phase*.08), canvas.height()*(.30-phase*.2)), font, 'Z' if index else 'z')
                painter.drawPath(letters)
            painter.restore()
