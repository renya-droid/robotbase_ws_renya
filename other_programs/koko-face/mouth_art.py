"""Smooth independent lips, individual teeth and a rooted animated tongue."""
import math
from pathlib import Path

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBitmap, QColor, QLinearGradient, QPainter, QPainterPath, QPen, QPixmap, QRegion


FRONT_TOOTH = QRectF(.470, .594, .028, .042)


def draw_tooth(painter, rect):
    painter.save()
    painter.setRenderHint(QPainter.Antialiasing)
    gradient = QLinearGradient(rect.topLeft(), rect.bottomLeft())
    gradient.setColorAt(0., QColor('#f5e9c8'))
    gradient.setColorAt(1., QColor('#d9c28b'))
    shape = QPainterPath(QPointF(rect.left(), rect.top()))
    shape.lineTo(rect.right(), rect.top())
    shape.lineTo(rect.right()-.04*rect.width(), rect.bottom()-.2*rect.height())
    shape.cubicTo(rect.right(), rect.bottom(), rect.left(), rect.bottom(),
                  rect.left()+.04*rect.width(), rect.bottom()-.2*rect.height())
    shape.closeSubpath()
    painter.setPen(QPen(QColor('#a5805d'), rect.width()*.025))
    painter.setBrush(gradient)
    painter.drawPath(shape)
    painter.setClipPath(shape)
    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor(141, 103, 44, 65))
    painter.drawEllipse(QRectF(rect.left()+rect.width()*.13, rect.top()+rect.height()*.12,
                              rect.width()*.24, rect.height()*.12))
    painter.restore()


class MouthArt:
    PROFILES = {
        'laughing': (.405, .585, .190, .152),
        'radiant': (.393, .589, .214, .140),
        'crying': (.465, .658, .070, .063),
        'shy': (.458, .661, .084, .041),
        'astonished': (.462, .578, .076, .126),
        'surprise': (.465, .655, .070, .082),
        'smile': (.447, .655, .106, .063),
        'sneezing': (.451, .655, .098, .095),
        'curious': (.467, .663, .066, .063),
        'excited': (.432, .655, .136, .092),
        'pout': (.456, .674, .088, .045),
        'relieved': (.453, .664, .094, .048),
        'dizzy': (.457, .660, .086, .067),
        'licking': (.443, .661, .114, .076),
    }

    def __init__(self):
        self.tongue = QPixmap(str(Path(__file__).parent/'assets'/'human_tongue.png'))
        if self.tongue.isNull():
            raise RuntimeError('舌の画像を読み込めません')
        bounds = QRegion(QBitmap.fromImage(self.tongue.toImage().createAlphaMask())).boundingRect()
        self.tongue = self.tongue.copy(bounds)

    @staticmethod
    def mouth_transform(painter, target, sample):
        painter.translate(target.x(), target.y())
        painter.scale(target.width(), target.height())
        center = (sample['mouth_top']+sample['mouth_bottom'])/2
        painter.translate(0., center*(1.-sample['mouth_open']))
        painter.scale(1., sample['mouth_open'])

    def draw(self, painter, target, sample):
        expression = sample['expression']
        left, top, width, height = self.PROFILES.get(expression, (.452, .666, .096, .060))
        happy = expression in ('laughing', 'radiant', 'happy', 'smile', 'playful', 'success', 'excited', 'relieved')
        sad = expression in ('crying', 'troubled', 'error', 'pout')
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing)
        if sample['mouth_open'] < .09:
            center = (sample['mouth_top']+sample['mouth_bottom'])/2
            curve = QPainterPath(QPointF(target.x()+left*target.width(),
                                        target.y()+center*target.height()))
            curve.quadTo(target.x()+(left+width/2)*target.width(),
                         target.y()+(center+(-.012 if sad else .014))*target.height(),
                         target.x()+(left+width)*target.width(), target.y()+center*target.height())
            painter.setPen(QPen(QColor('#b77d70'), max(1.6, target.height()*.0038),
                                Qt.SolidLine, Qt.RoundCap))
            painter.setBrush(Qt.NoBrush)
            painter.drawPath(curve)
            painter.restore()
            return
        self.mouth_transform(painter, target, sample)
        mouth = QPainterPath(QPointF(left, top))
        mouth.quadTo(left+width*.5, top+(.012 if happy else -.007), left+width, top)
        mouth.cubicTo(left+width*.94, top+height*1.18,
                      left+width*.06, top+height*1.18, left, top)
        mouth.closeSubpath()
        depth = QLinearGradient(QPointF(.5, top), QPointF(.5, top+height))
        depth.setColorAt(0., QColor('#3e1c1d'))
        depth.setColorAt(.6, QColor('#672d2e'))
        depth.setColorAt(1., QColor('#9c514b'))
        painter.setBrush(depth)
        painter.setPen(QPen(QColor('#ae7164'), .0028, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        painter.drawPath(mouth)
        painter.setClipPath(mouth)
        if happy:
            row_left, step, row_top, tooth_height = (
                (.414, .028, .594, .042) if expression == 'laughing'
                else (left+width*.04, width*.92/6, top+.009, min(.046, height*.40)))
            for index in range(6):
                if index == 3 or (expression == 'laughing' and index == 2 and sample.get('missing_tooth')):
                    continue
                draw_tooth(painter, QRectF(row_left+step*index+.0007, row_top,
                                          step-.0014, tooth_height))
        if expression != 'licking':
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor('#bb706c'))
            painter.drawEllipse(QRectF(left+width*.24, top+height*.78, width*.52, height*.32))
        painter.restore()

    def draw_tongue(self, painter, target, sample):
        if sample['expression'] != 'licking':
            return
        t = sample['elapsed']
        extension = (.050 + .092*(.5+.5*math.sin(t*4.4))) * min(1., sample['mouth_open']/.3)
        root = .5*(sample['mouth_top']+sample['mouth_bottom'])
        root += (.682-root)*sample['mouth_open']
        width = (.062 + .008*math.sin(t*4.4+.6)) * min(1., sample['mouth_open']/.3)
        if sample['mouth_open'] < .09:
            return
        painter.save()
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        # A fixed root and smoothly bending tip, rather than shearing a mouth GIF.
        for row in range(32):
            y0, y1 = row/32, (row+1)/32
            bend = .009*math.sin(t*4.4+.3)*((y0+y1)/2)**1.6
            painter.drawPixmap(QRectF(target.x()+(.5-width/2+bend)*target.width(),
                                     target.y()+(root+extension*y0)*target.height(),
                                     width*target.width(), extension*(y1-y0)*target.height()+.12),
                               self.tongue, QRectF(0., self.tongue.height()*y0,
                                                   self.tongue.width(), self.tongue.height()*(y1-y0)))
        painter.restore()
