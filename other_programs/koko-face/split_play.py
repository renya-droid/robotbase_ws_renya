"""Small bouncing companions that split on touch and merge after inactivity."""
import math
import time
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont


class SplitPlay:
    EMOTIONS = ('radiant', 'crying', 'pout', 'shy', 'astonished',
                'curious', 'relieved', 'excited', 'dizzy', 'laughing')
    MAX_FACES = 8
    IDLE_SECONDS = 5.
    MERGE_INTERVAL = 5.
    MERGE_SECONDS = 2.8
    COLLISION_GRACE = 1.

    def __init__(self):
        self.active = False
        self.faces = []
        self.last_touch = self.last_step = 0.
        self.merge_started = None
        self.merging_ids = ()
        self.next_merge_at = None
        self.face_fraction = (1., 1.)
        self.serial = 0
        self.contacts = set()
        self.pending_collisions = []
        self.drag_id = None
        self.social_next = 0.

    def _choose_emotion(self, serial):
        occupied = {face['emotion'] for face in self.faces}
        choices = [emotion for emotion in self.EMOTIONS if emotion not in occupied]
        return choices[serial % len(choices)]

    @staticmethod
    def _emotion_duration(serial):
        return 3.3+(serial % 5)*.61

    def _face(self, x, y, scale, target, vx, vy):
        self.serial += 1
        return dict(x=x, y=y, scale=scale, target=target, vx=vx, vy=vy,
                    phase=self.serial*2.4, angle=0., opacity=1., id=self.serial,
                    emotion=self._choose_emotion(self.serial-1), previous_emotion=None,
                    emotion_since=self.last_step, next_emotion=self.last_step+self._emotion_duration(self.serial),
                    merge_blend=0.)

    def start(self, now=None):
        now = time.monotonic() if now is None else now
        self.active = True
        self.drag_id = None
        self.social_next = now+1.8
        self.last_touch = self.last_step = now
        self.merge_started = None
        self.merging_ids = ()
        self.next_merge_at = now+self.IDLE_SECONDS
        self.faces = []
        self.faces.append(self._face(.5, .5, 1., .52, -.13, -.065))
        self.faces.append(self._face(.5, .5, 1., .52, .13, .065))
        self.contacts = self._contacts()
        self.pending_collisions = []

    def stop(self):
        self.active = False
        self.drag_id = None
        self.faces = []
        self.merge_started = None
        self.merging_ids = ()
        self.next_merge_at = None
        self.contacts = set()
        self.pending_collisions = []

    def touch(self, x, y, now=None):
        if not self.active:
            return False
        now = time.monotonic() if now is None else now
        self.last_touch = self.last_step = now
        self.merge_started = None
        self.merging_ids = ()
        self.next_merge_at = now+self.IDLE_SECONDS
        for face in self.faces:
            face.update(opacity=1., merge_blend=0.)
        nearest = min(self.faces, key=lambda face: (face['x']-x)**2+(face['y']-y)**2)
        if len(self.faces) < self.MAX_FACES:
            self.faces.remove(nearest)
            target = max(.23, nearest['target']*.78)
            direction = self.serial*2.399
            for side in (-1., 1.):
                self.faces.append(self._face(nearest['x'], nearest['y'], nearest['scale'], target,
                                             side*.18*math.cos(direction), side*.13*math.sin(direction)))
        else:
            # At the limit, tapping scatters the companions instead of allocating more.
            for face in self.faces:
                angle = math.atan2(face['y']-y, face['x']-x)
                face['vx'], face['vy'] = .18*math.cos(angle), .14*math.sin(angle)
        self.contacts = self._contacts()
        self.pending_collisions = []
        return True

    def merge(self, now=None, pair=None):
        """Start one pair; remaining companions continue playing until their turn."""
        if not self.active or self.merge_started is not None or self.drag_id is not None:
            return
        self.merge_started = time.monotonic() if now is None else now
        receiver = self.faces[0] if pair is None else pair[0]
        donor = (min(self.faces[1:], key=lambda face: (face['x']-receiver['x'])**2+
                                                   (face['y']-receiver['y'])**2)
                 if pair is None else pair[1])
        self.merging_ids = (receiver['id'], donor['id'])
        self.merge_final = len(self.faces) == 2
        size = 1. if self.merge_final else min(.86, math.hypot(receiver['target'], donor['target']))
        x, y = (.5, .5) if self.merge_final else (receiver['x'], receiver['y'])
        x = max(.27*size*self.face_fraction[0], min(1.-.27*size*self.face_fraction[0], x))
        y = max(.49*size*self.face_fraction[1], min(1.-.49*size*self.face_fraction[1], y))
        self.merge_to = x, y, size
        for face in (receiver, donor):
            face['merge_from'] = (face['x'], face['y'], face['scale'], face['angle'])

    def _contacts(self):
        """Test the visible oval heads, rather than their transparent canvases."""
        contacts = set()
        if min(self.face_fraction) <= 0.:
            return contacts
        for index, first in enumerate(self.faces):
            for second in self.faces[index+1:]:
                size = first['scale']+second['scale']
                radius_x = .25*size*self.face_fraction[0]
                radius_y = .45*size*self.face_fraction[1]
                if ((first['x']-second['x'])/radius_x)**2+((first['y']-second['y'])/radius_y)**2 <= 1.:
                    contacts.add(tuple(sorted((first['id'], second['id']))))
        return contacts

    def _merge_collisions(self, now):
        contacts = self._contacts()
        if now-self.last_touch >= self.COLLISION_GRACE:
            for pair in sorted(contacts-self.contacts):
                if not any(identity in self.merging_ids for identity in pair):
                    self.pending_collisions.append(pair)
        self.contacts = contacts
        if self.merge_started is not None:
            return
        live = {face['id']: face for face in self.faces}
        while self.pending_collisions:
            pair = self.pending_collisions.pop(0)
            if pair in contacts and all(identity in live for identity in pair):
                self.merge(now, (live[pair[0]], live[pair[1]]))
                break

    def _advance_merge(self, now):
        while self.active:
            if self.merge_started is None:
                if now < self.next_merge_at:
                    return False
                self.merge(self.next_merge_at)
            progress = max(0., min(1., (now-self.merge_started)/self.MERGE_SECONDS))
            gather = min(1., progress/.65)
            blend = gather*gather*(3.-2.*gather)
            growth = max(0., min(1., (progress-.55)/.45))
            growth = growth*growth*(3.-2.*growth)
            receiver, donor = (next(face for face in self.faces if face['id'] == identity)
                               for identity in self.merging_ids)
            dest_x, dest_y, dest_size = self.merge_to
            for face in (receiver, donor):
                x, y, scale, angle = face['merge_from']
                receiving = face is receiver
                face.update(x=x*(1.-blend)+dest_x*blend, y=y*(1.-blend)+dest_y*blend,
                            scale=scale*(1.-growth)+dest_size*growth if receiving else scale,
                            angle=angle*(1.-blend), opacity=1. if receiving else 1.-growth,
                            merge_blend=growth if receiving and self.merge_final else 0.)
            if progress < 1.-1e-9:
                return False
            if self.merge_final:
                self.stop()
                return True
            self.faces.remove(donor)
            receiver.update(target=dest_size, opacity=1., merge_blend=0.)
            self.next_merge_at = self.merge_started+self.MERGE_INTERVAL
            self.merge_started = None
            self.merging_ids = ()
        return False

    def advance(self, now=None, face_fraction=(1., 1.)):
        if not self.active:
            return False
        now = time.monotonic() if now is None else now
        dt = max(0., min(.05, now-self.last_step))
        self.last_step = now
        self.face_fraction = face_fraction
        if self.drag_id is not None:
            self.next_merge_at = now+self.IDLE_SECONDS
        if self.drag_id is None and self._advance_merge(now):
            return True
        for face in self.faces:
            if face['id'] in self.merging_ids or face['id'] == self.drag_id:
                continue
            if now >= face['next_emotion']:
                face['previous_emotion'] = face['emotion']
                face['emotion'] = self._choose_emotion(face['id'])
                face['emotion_since'] = now
                face['next_emotion'] = now+self._emotion_duration(face['id'])
            face['merge_blend'] = 0.
            face['opacity'] = 1.
            face['scale'] += (face['target']-face['scale'])*(1.-math.exp(-dt*6.))
            face['x'] += face['vx']*dt
            face['y'] += face['vy']*dt
            for axis, velocity, margin in (('x', 'vx', .27*face['scale']*face_fraction[0]),
                                           ('y', 'vy', .49*face['scale']*face_fraction[1])):
                if face[axis] < margin:
                    face[axis], face[velocity] = margin, abs(face[velocity])
                elif face[axis] > 1.-margin:
                    face[axis], face[velocity] = 1.-margin, -abs(face[velocity])
            energy = {'crying': 1.8, 'relieved': 1.3, 'shy': 2., 'excited': 4.}.get(face['emotion'], 3.)
            face['angle'] = energy*math.sin(now*2.+face['phase'])
            if face['emotion'] == 'crying':
                face['angle'] += .6*math.sin(now*13.+face['phase'])
            elif face['emotion'] == 'shy':
                face['angle'] -= 1.5
        if self.drag_id is None:
            self._social_reaction(now)
            self._merge_collisions(now)
        return False

    def drag_to(self, start_x, start_y, x, y, now):
        if not self.active:
            return
        if self.drag_id is None:
            face = min(self.faces, key=lambda f: (f['x']-start_x)**2+(f['y']-start_y)**2)
            self.drag_id = face['id']
            self.merge_started = None
            self.merging_ids = ()
            for item in self.faces:
                item.update(opacity=1., merge_blend=0.)
            self.drag_time = now-.033
        face = next(f for f in self.faces if f['id'] == self.drag_id)
        dt = max(.016, now-self.drag_time)
        margin_x, margin_y = .27*face['scale']*self.face_fraction[0], .49*face['scale']*self.face_fraction[1]
        x, y = max(margin_x, min(1.-margin_x, x)), max(margin_y, min(1.-margin_y, y))
        face.update(vx=max(-.8, min(.8, (x-face['x'])/dt)),
                    vy=max(-.8, min(.8, (y-face['y'])/dt)), x=x, y=y)
        self.drag_time = now
        self.last_touch = now
        self.next_merge_at = now+self.IDLE_SECONDS

    def release_drag(self, now):
        self.drag_id = None
        self.last_touch = now
        self.next_merge_at = now+self.IDLE_SECONDS
        self.contacts = self._contacts()
        self.pending_collisions = []

    def _social_reaction(self, now):
        if now < self.social_next:
            return
        self.social_next = now+2.6
        free = [f for f in self.faces if f['id'] not in self.merging_ids]
        for source in free:
            listeners = [f for f in free if f is not source and
                         math.hypot(f['x']-source['x'], f['y']-source['y']) < .48]
            if not listeners:
                continue
            listener = min(listeners, key=lambda f: (f['x']-source['x'])**2+(f['y']-source['y'])**2)
            if source['emotion'] == 'crying':
                source.update(social_emotion=None, social_text='ぐすん…', social_until=now+2.4)
                listener.update(social_emotion='shy', social_text='だいじょうぶ？', social_until=now+2.4,
                                social_gaze=max(-1., min(1., (source['x']-listener['x'])*5.)))
                # Approach gently without changing the existing collision/merge rules.
                listener['vx'] = (source['x']-listener['x'])*.18
                listener['vy'] = (source['y']-listener['y'])*.18
                return
            if source['emotion'] in ('radiant', 'laughing', 'excited'):
                source.update(social_emotion=None, social_text='あはは！', social_until=now+2.4)
                listener.update(social_emotion='laughing', social_text='つられちゃった！', social_until=now+2.4,
                                social_gaze=max(-1., min(1., (source['x']-listener['x'])*5.)))
                return

    def paint(self, painter, canvas, target, sprite, draw_contour=None):
        for face in self.faces:
            if face['opacity'] < .005:
                continue
            image = sprite(face) if callable(sprite) else sprite
            painter.save()
            painter.setOpacity(face['opacity'])
            painter.translate(canvas.x()+face['x']*canvas.width(), canvas.y()+face['y']*canvas.height())
            painter.rotate(face['angle'])
            width, height = target.width()*face['scale'], target.height()*face['scale']
            painter.drawPixmap(QRectF(-width/2, -height/2, width, height), image, QRectF(image.rect()))
            if draw_contour is not None:
                draw_contour(painter, QRectF(-width/2, -height/2, width, height))
            painter.restore()
            if self.last_step < face.get('social_until', 0.):
                text = face.get('social_text', '')
                painter.save()
                painter.setFont(QFont('Sans Serif', max(10, int(canvas.height()*.022))))
                metrics = painter.fontMetrics()
                bubble = QRectF(0., 0., metrics.horizontalAdvance(text)+16., metrics.height()+12.)
                bubble.moveCenter(QPointF(max(bubble.width()/2, min(canvas.width()-bubble.width()/2,
                                         face['x']*canvas.width())),
                                         max(bubble.height()/2, face['y']*canvas.height()-height*.43)))
                painter.setPen(QColor('#b88f6c')); painter.setBrush(QColor('#fff4dc'))
                painter.drawRoundedRect(bubble, 8., 8.)
                painter.setPen(QColor('#4b342b')); painter.drawText(bubble, Qt.AlignCenter, text)
                painter.restore()
