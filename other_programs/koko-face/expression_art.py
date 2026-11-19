"""Code-native expressions sharing Koko's original nose, hair and face outline."""
import math
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QLinearGradient, QPainter, QPainterPath, QPen

EXTRA_EXPRESSIONS = ('curious', 'excited', 'pout', 'relieved', 'dizzy')
NATIVE_EXPRESSIONS = (*EXTRA_EXPRESSIONS, 'sneezing')


def draw_eyebrow(painter, state, side, cx, brow_y=.270):
    """A tapered hair silhouette, with paint state isolated from the eyes."""
    painter.save()
    painter.translate(cx, brow_y + (-.023 if state == 'curious' and side == 0 else
                                  -.010 if state in ('astonished', 'surprise') else 0.))
    painter.scale(1. if side == 0 else -1., 1.)
    painter.rotate(13. if state in ('pout', 'sneezing', 'error') else -8. if state == 'curious' and side == 0
                   else -15. if state in ('crying', 'troubled')
                   else -5. if state in ('relieved', 'sleep', 'shy') else 0.)
    brow = QPainterPath(QPointF(-.058, .012))
    brow.cubicTo(-.040, -.013, -.007, -.035, .030, -.029)
    brow.cubicTo(.050, -.026, .061, -.010, .055, .012)
    brow.quadTo(.052, .024, .043, .027)
    brow.cubicTo(.011, .004, -.024, .001, -.058, .012)
    brow.closeSubpath()
    shade = QLinearGradient(QPointF(0., -.035), QPointF(0., .027))
    shade.setColorAt(0., QColor('#5c534c'))
    shade.setColorAt(1., QColor('#423c38'))
    painter.setPen(Qt.NoPen)
    painter.setBrush(shade)
    painter.drawPath(brow)
    painter.setClipPath(brow)
    painter.setBrush(Qt.NoBrush)
    painter.setPen(QPen(QColor(141, 123, 106, 60), .0013, Qt.SolidLine, Qt.RoundCap))
    for offset in (-.022, -.003, .017):
        strand = QPainterPath(QPointF(offset-.014, -.016))
        strand.quadTo(offset+.004, -.024, offset+.018, -.006)
        painter.drawPath(strand)
    painter.restore()


def draw_expression_eyes(painter, target, sample):
    state = sample['expression']
    painter.save()
    painter.translate(target.x(), target.y())
    painter.scale(target.width(), target.height())
    painter.setRenderHint(QPainter.Antialiasing)
    # Clear eyebrows separately and leave the nose bridge between the eyes.
    eye_rects = ((QRectF(.33, .300, .16, .125), QRectF(.51, .300, .16, .125))
                 if state == 'error' else
                 (QRectF(.34, .322, .14, .103), QRectF(.52, .322, .14, .103)))
    clear = QPainterPath()
    for rect in eye_rects:
        clear.addRect(rect)
    if state == 'error':
        nose = QPainterPath()
        nose.addEllipse(QRectF(.45, .39, .10, .18))
        clear = clear.subtracted(nose)
    painter.fillPath(clear, QColor(Qt.black))
    gaze_x, gaze_y = sample.get('gaze', (0., 0.))
    t = sample['elapsed']
    for side, cx in enumerate((.417, .583)):
        factor = sample['eye_factor']
        if state == 'relieved' or factor < .15 or (state == 'sneezing' and factor < .5):
            path = QPainterPath(QPointF(cx-.042, .368))
            path.quadTo(cx, .397, cx+.042, .368)
            painter.setPen(QPen(QColor('#fff2dc'), .006, Qt.SolidLine, Qt.RoundCap))
            painter.setBrush(Qt.NoBrush)
            painter.drawPath(path)
            continue
        height = (.080 if state == 'excited' else .062 if state == 'curious' else .045)
        height *= factor
        rect = QRectF(cx-.045, .367-height/2, .09, height)
        painter.setPen(QPen(QColor('#826a58'), .002))
        painter.setBrush(QColor('#fff7e9'))
        painter.drawEllipse(rect)
        painter.save()
        clip = QPainterPath(); clip.addEllipse(rect); painter.setClipPath(clip)
        if state == 'dizzy':
            spiral = QPainterPath()
            for step in range(90):
                angle = step*.19+t*1.8
                radius = .001+step*.00030
                point = QPointF(cx+math.cos(angle)*radius, .367+math.sin(angle)*radius*.8)
                if step == 0: spiral.moveTo(point)
                else: spiral.lineTo(point)
            painter.setPen(QPen(QColor('#604136'), .0035))
            painter.setBrush(Qt.NoBrush); painter.drawPath(spiral)
        else:
            cross = sample.get('cross_eye', 0.)
            inward = .025 if side == 0 else -.025
            x = cx+.016*gaze_x*(1.-cross)+inward*cross
            y = .367+.010*gaze_y*(1.-cross)+.010*cross
            painter.setPen(Qt.NoPen); painter.setBrush(QColor('#624536'))
            painter.drawEllipse(QRectF(x-.018, y-.023, .036, .046))
            painter.setBrush(QColor('#231b19'))
            painter.drawEllipse(QRectF(x-.009, y-.014, .018, .028))
            painter.setBrush(Qt.white)
            painter.drawEllipse(QRectF(x-.009, y-.015, .009, .012))
        painter.restore()
    if state in ('excited', 'pout'):
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(222, 119, 111, 90))
        for x in (.352, .616):
            painter.drawEllipse(QRectF(x, .481, .032, .024))
    painter.restore()
