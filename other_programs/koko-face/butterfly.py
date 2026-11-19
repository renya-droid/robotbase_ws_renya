"""A silent screen-space butterfly that the face can notice and chase."""
import math
import time
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QLinearGradient, QPainter, QPainterPath, QPen


class Butterfly:
    DURATION = 18.
    LAND_START, LANDED, INHALE, SNEEZE, RECOVER = 7., 8.8, 10.5, 11.2, 12.2

    def __init__(self):
        self.started = None

    @property
    def active(self):
        return self.started is not None

    def start(self, now=None):
        self.started = time.monotonic() if now is None else now

    def stop(self):
        self.started = None

    def elapsed(self, now=None):
        return 0. if not self.active else round((time.monotonic() if now is None else now)-self.started, 6)

    @staticmethod
    def position(elapsed):
        t = max(0., elapsed)
        x = .5+.31*math.sin(t*.63-.7)
        y = .31+.14*math.sin(t*1.13)+.035*math.sin(t*3.1)
        if t < 2.:
            blend = (1.-t/2.)**2
            x = x*(1.-blend)-.08*blend
        if t > 15.:
            blend = min(1., (t-15.)/3.)**2
            x = x*(1.-blend)+1.08*blend
        return QPointF(x, y)

    @staticmethod
    def ease(value):
        value = max(0., min(1., value))
        return value*value*(3.-2.*value)

    @classmethod
    def expression(cls, t):
        if t < 2.2: return 'curious'
        if t < cls.LAND_START: return 'excited'
        if t < cls.INHALE: return 'curious'
        if t < cls.RECOVER: return 'sneezing'
        return 'curious' if t < 14.5 else 'relieved'

    @classmethod
    def landing_blend(cls, t):
        return cls.ease((t-cls.LAND_START)/(cls.LANDED-cls.LAND_START))

    @classmethod
    def chase_position(cls, t):
        if t < cls.LAND_START:
            return cls.position(t)
        rest = cls.position(cls.LAND_START)
        if t < cls.SNEEZE:
            return rest
        escape = cls.ease((t-cls.SNEEZE)/1.3)
        point = rest*(1.-escape)+QPointF(.92, .12)*escape
        settle = cls.ease((t-14.5)/2.5)
        return point*(1.-settle)+QPointF(.5, .40)*settle

    @staticmethod
    def size(canvas):
        return max(16., min(46., canvas.height()*.059))

    @classmethod
    def screen_position(cls, canvas, t, nose_anchor=None):
        point = cls.position(t)
        flying = QPointF(canvas.x()+point.x()*canvas.width(), canvas.y()+point.y()*canvas.height())
        if nose_anchor is None:
            return flying
        perch = nose_anchor-QPointF(0., cls.size(canvas)*.46)
        if t >= cls.SNEEZE:
            launch = t-cls.SNEEZE
            return perch+QPointF(canvas.width()*.60*launch**1.35,
                                  -canvas.height()*.40*launch**1.15)
        blend = cls.landing_blend(t)
        return flying*(1.-blend)+perch*blend

    def paint_sneeze(self, painter, canvas, mouth_anchor, t):
        burst = t-self.SNEEZE
        if mouth_anchor is None or not 0. <= burst < .65:
            return
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(242, 240, 217, round(200*(1.-burst/.65))),
                            max(1., canvas.height()*.003), Qt.SolidLine, Qt.RoundCap))
        length = canvas.height()*(.025+.11*burst)
        for direction in (-1., 0., 1.):
            start = mouth_anchor+QPointF(direction*length*.22, length*.10)
            end = mouth_anchor+QPointF(direction*length*.85, length)
            painter.drawLine(start, end)
        painter.restore()

    def paint(self, painter, canvas, now=None, nose_anchor=None, mouth_anchor=None):
        if not self.active:
            return
        t = self.elapsed(now)
        if t < 0. or t >= self.DURATION:
            return
        point = self.screen_position(canvas, t, nose_anchor)
        ahead = self.position(t+.025)-self.position(t)
        self.paint_sneeze(painter, canvas, mouth_anchor, t)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(point)
        size = self.size(canvas)
        painter.rotate(-22. if t >= self.SNEEZE else
                       math.degrees(math.atan2(ahead.y(), ahead.x()))*.18*(1.-self.landing_blend(t)))
        painter.scale(size, size)
        landed = self.LANDED <= t < self.SNEEZE
        flap = (.62+.09*math.sin(t*3.)) if landed else .22+.78*abs(math.sin(t*math.tau*5.5))
        gradient = QLinearGradient(QPointF(0., -.8), QPointF(0., .7))
        gradient.setColorAt(0., QColor('#ffc06a')); gradient.setColorAt(1., QColor('#d256ad'))
        for side in (-1., 1.):
            painter.save(); painter.scale(side*flap, 1.)
            wing = QPainterPath(QPointF(0., 0.))
            wing.cubicTo(.55, -1.15, 1.25, -.9, .88, -.13)
            wing.cubicTo(1.06, .65, .38, .90, 0., .08)
            painter.setPen(QPen(QColor('#6c3557'), .045)); painter.setBrush(gradient)
            painter.drawPath(wing)
            painter.setPen(Qt.NoPen); painter.setBrush(QColor('#fff1ca'))
            for x, y in ((.65, -.48), (.52, .34)):
                painter.drawEllipse(QPointF(x, y), .10, .11)
            painter.restore()
        painter.setPen(QPen(QColor('#523646'), .085, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(0., -.40), QPointF(0., .46))
        painter.setPen(QPen(QColor('#d2a88c'), .027))
        for side in (-1., 1.):
            painter.drawLine(QPointF(0., -.32), QPointF(side*.20, -.65))
        painter.restore()
