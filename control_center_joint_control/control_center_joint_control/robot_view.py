"""Lightweight URDF/STL robot viewer embedded directly in the control panel."""

from dataclasses import dataclass
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .joint_definitions import GRIPPER_JOINTS, JOINT_LABELS, JOINT_LIMITS

try:
    from OpenGL.GL import (
        GL_AMBIENT_AND_DIFFUSE,
        GL_BLEND,
        GL_COLOR_BUFFER_BIT,
        GL_COLOR_MATERIAL,
        GL_DEPTH_BUFFER_BIT,
        GL_DEPTH_TEST,
        GL_FLOAT,
        GL_FRAMEBUFFER,
        GL_FRONT_AND_BACK,
        GL_LIGHT0,
        GL_LIGHTING,
        GL_LINES,
        GL_MODELVIEW,
        GL_NORMAL_ARRAY,
        GL_NORMALIZE,
        GL_ONE_MINUS_SRC_ALPHA,
        GL_POSITION,
        GL_PROJECTION,
        GL_RGB,
        GL_SMOOTH,
        GL_SRC_ALPHA,
        GL_TRIANGLES,
        GL_UNSIGNED_BYTE,
        GL_VERTEX_ARRAY,
        glBegin,
        glBindFramebuffer,
        glBlendFunc,
        glClear,
        glClearColor,
        glColor4f,
        glColor3ub,
        glColorMaterial,
        glDisable,
        glDisableClientState,
        glDrawArrays,
        glEnable,
        glEnableClientState,
        glEnd,
        glFrustum,
        glLightfv,
        glLineWidth,
        glLoadIdentity,
        glMatrixMode,
        glMultMatrixf,
        glNormalPointer,
        glPopMatrix,
        glPushMatrix,
        glReadPixels,
        glShadeModel,
        glVertex3f,
        glVertexPointer,
        glViewport,
    )
    from OpenGL.GLU import gluLookAt
    OPENGL_ERROR = ''
except Exception as error:  # pragma: no cover - only used for dependency diagnostics
    OPENGL_ERROR = str(error)


def _numbers(text, count, default):
    if not text:
        return np.array(default, dtype=np.float64)
    values = [float(item) for item in text.split()]
    if len(values) != count:
        raise ValueError(f'expected {count} numbers, got {values}')
    return np.array(values, dtype=np.float64)


def _origin(element):
    if element is None:
        return np.eye(4, dtype=np.float64)
    xyz = _numbers(element.get('xyz'), 3, (0.0, 0.0, 0.0))
    roll, pitch, yaw = _numbers(
        element.get('rpy'), 3, (0.0, 0.0, 0.0)
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    length = np.linalg.norm(axis)
    if length < 1e-12:
        return np.eye(4, dtype=np.float64)
    x, y, z = axis / length
    c, s, one_c = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    rotation = np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ])
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    return result


def _rotation_guide_geometry(origin, axis, radius=0.11, steps=28):
    """Return right-hand-rule arcs and arrowheads for a revolute joint."""
    origin = np.asarray(origin, dtype=np.float64)
    axis = np.asarray(axis, dtype=np.float64)
    length = np.linalg.norm(axis)
    if length < 1e-12:
        raise ValueError('joint guide axis must be non-zero')
    axis = axis / length
    references = np.eye(3, dtype=np.float64)
    reference = references[np.argmin(np.abs(references @ axis))]
    radial = np.cross(axis, reference)
    radial /= np.linalg.norm(radial)
    tangent = np.cross(axis, radial)

    def arc(sign):
        angles = np.linspace(0.0, sign * math.radians(145.0), steps)
        points = np.array([
            origin + radius * (
                math.cos(angle) * radial + math.sin(angle) * tangent
            )
            for angle in angles
        ])
        angle = angles[-1]
        endpoint = points[-1]
        direction = sign * (
            -math.sin(angle) * radial + math.cos(angle) * tangent
        )
        side = np.cross(axis, direction)
        arrow_length = radius * 0.28
        arrow_width = radius * 0.15
        arrowheads = (
            (endpoint, endpoint - direction * arrow_length + side * arrow_width),
            (endpoint, endpoint - direction * arrow_length - side * arrow_width),
        )
        return points, arrowheads

    positive_arc, positive_arrowheads = arc(1.0)
    negative_arc, negative_arrowheads = arc(-1.0)
    return {
        'axis': (
            origin - axis * radius * 0.85,
            origin + axis * radius * 0.85,
        ),
        'positive_arc': positive_arc,
        'positive_arrowheads': positive_arrowheads,
        'negative_arc': negative_arc,
        'negative_arrowheads': negative_arrowheads,
    }


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass
class Visual:
    link: str
    mesh_path: Path
    origin: np.ndarray
    scale: np.ndarray
    color: tuple


class RobotModel:
    def __init__(self, urdf_path):
        self.urdf_path = Path(urdf_path).resolve()
        root = ET.parse(self.urdf_path).getroot()
        links = {element.get('name') for element in root.findall('link')}
        child_links = set()
        self.children = {}
        self.parent_joint = {}
        self.joints_by_name = {}
        self._selectable_joint_cache = {}
        self.visuals = []

        for element in root.findall('joint'):
            parent = element.find('parent').get('link')
            child = element.find('child').get('link')
            child_links.add(child)
            joint = Joint(
                name=element.get('name'),
                joint_type=element.get('type', 'fixed'),
                parent=parent,
                child=child,
                origin=_origin(element.find('origin')),
                axis=_numbers(
                    None if element.find('axis') is None
                    else element.find('axis').get('xyz'),
                    3,
                    (0.0, 0.0, 0.0),
                ),
            )
            self.children.setdefault(parent, []).append(joint)
            self.parent_joint[child] = joint
            self.joints_by_name[joint.name] = joint
        roots = sorted(links - child_links)
        if not roots:
            raise ValueError('URDF has no root link')
        self.root_link = roots[0]

        for link in root.findall('link'):
            link_name = link.get('name')
            for visual in link.findall('visual'):
                mesh = visual.find('./geometry/mesh')
                if mesh is None:
                    continue
                filename = mesh.get('filename', '')
                if filename.startswith('package://'):
                    filename = filename.split('/', 3)[-1]
                    mesh_path = self.urdf_path.parent.parent / filename
                else:
                    mesh_path = self.urdf_path.parent / filename
                color_element = visual.find('./material/color')
                rgba = _numbers(
                    None if color_element is None else color_element.get('rgba'),
                    4,
                    (0.72, 0.76, 0.84, 1.0),
                )
                scale = _numbers(mesh.get('scale'), 3, (1.0, 1.0, 1.0))
                self.visuals.append(Visual(
                    link=link_name,
                    mesh_path=mesh_path.resolve(),
                    origin=_origin(visual.find('origin')),
                    scale=scale,
                    color=tuple(float(value) for value in rgba),
                ))

    def selectable_joint_for_link(self, link):
        if link in self._selectable_joint_cache:
            return self._selectable_joint_cache[link]
        cursor = link
        selected = None
        while cursor in self.parent_joint:
            joint = self.parent_joint[cursor]
            if joint.name in JOINT_LIMITS and joint.name not in GRIPPER_JOINTS:
                selected = joint.name
                break
            cursor = joint.parent
        self._selectable_joint_cache[link] = selected
        return selected

    def link_transforms(self, joint_degrees):
        transforms = {self.root_link: np.eye(4, dtype=np.float64)}
        pending = [self.root_link]
        while pending:
            parent = pending.pop()
            parent_transform = transforms[parent]
            for joint in self.children.get(parent, []):
                value = math.radians(float(joint_degrees.get(joint.name, 0.0)))
                motion = np.eye(4, dtype=np.float64)
                if joint.joint_type in ('revolute', 'continuous'):
                    motion = _axis_rotation(joint.axis, value)
                elif joint.joint_type == 'prismatic':
                    # UI values are degrees for robot joints; the three virtual
                    # ground joints are always absent/zero in this viewer.
                    motion[:3, 3] = joint.axis * float(
                        joint_degrees.get(joint.name, 0.0)
                    )
                transforms[joint.child] = parent_transform @ joint.origin @ motion
                pending.append(joint.child)
        return transforms

    def joint_world_axis(self, name, joint_degrees):
        joint = self.joints_by_name.get(name)
        if joint is None or joint.joint_type not in ('revolute', 'continuous'):
            return None
        transforms = self.link_transforms(joint_degrees)
        parent_transform = transforms.get(joint.parent)
        if parent_transform is None:
            return None
        frame = parent_transform @ joint.origin
        axis = frame[:3, :3] @ joint.axis
        length = np.linalg.norm(axis)
        if length < 1e-12:
            return None
        return frame[:3, 3], axis / length


def _load_binary_stl(path):
    path = Path(path)
    with path.open('rb') as stream:
        header = stream.read(84)
    if len(header) != 84:
        raise ValueError(f'invalid STL header: {path}')
    triangle_count = struct.unpack('<I', header[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if path.stat().st_size != expected_size:
        raise ValueError(f'only binary STL is supported: {path}')
    dtype = np.dtype([
        ('normal', '<f4', (3,)),
        ('vertices', '<f4', (3, 3)),
        ('attribute', '<u2'),
    ])
    triangles = np.memmap(
        path, dtype=dtype, mode='r', offset=84, shape=(triangle_count,)
    )
    vertices = np.ascontiguousarray(
        triangles['vertices'].reshape(-1, 3), dtype=np.float32
    )
    normals = np.ascontiguousarray(
        np.repeat(triangles['normal'], 3, axis=0), dtype=np.float32
    )
    return vertices, normals


class RobotOpenGLWidget(QtWidgets.QOpenGLWidget):
    joint_selected = QtCore.pyqtSignal(str)
    joint_target_dragged = QtCore.pyqtSignal(str, float)

    def __init__(self, urdf_path, parent=None):
        super().__init__(parent)
        surface = QtGui.QSurfaceFormat()
        surface.setRenderableType(QtGui.QSurfaceFormat.OpenGL)
        surface.setProfile(QtGui.QSurfaceFormat.CompatibilityProfile)
        surface.setVersion(2, 1)
        surface.setDepthBufferSize(24)
        # A single-sample framebuffer keeps color-ID picking portable across
        # Mesa and vendor drivers. Smooth lighting still gives clean visuals.
        surface.setSamples(0)
        self.setFormat(surface)
        self.setMinimumSize(430, 420)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMouseTracking(True)

        self.model = RobotModel(urdf_path)
        self.meshes = {}
        self.current_degrees = {}
        self.target_degrees = {}
        self.show_target = True
        self.target_opacity = 0.28
        self.interaction_mode = 'camera'
        self.selected_joint = ''
        self.connected = False
        self.render_error = OPENGL_ERROR
        self.yaw = math.radians(42.0)
        self.pitch = math.radians(18.0)
        self.distance = 2.75
        self.center = np.array((0.0, 0.0, 0.72), dtype=np.float64)
        self._last_mouse = None
        self._camera_drag_kind = None
        self._joint_drag_start_x = None
        self._joint_drag_start_value = None

    def initializeGL(self):
        if self.render_error:
            return
        try:
            glClearColor(0.075, 0.086, 0.105, 1.0)
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_NORMALIZE)
            glEnable(GL_COLOR_MATERIAL)
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
            glShadeModel(GL_SMOOTH)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glLightfv(GL_LIGHT0, GL_POSITION, (3.0, -4.0, 6.0, 1.0))
            for visual in self.model.visuals:
                path = str(visual.mesh_path)
                if path not in self.meshes:
                    self.meshes[path] = _load_binary_stl(visual.mesh_path)
        except Exception as error:
            self.render_error = str(error)

    def resizeGL(self, width, height):
        if self.render_error:
            return
        glViewport(0, 0, max(1, width), max(1, height))

    def paintGL(self):
        if self.render_error:
            painter = QtGui.QPainter(self)
            painter.fillRect(self.rect(), QtGui.QColor('#15181e'))
            painter.setPen(QtGui.QColor('#ffad66'))
            painter.drawText(
                self.rect().adjusted(24, 24, -24, -24),
                QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap,
                '三维视图初始化失败\n' + self.render_error,
            )
            painter.end()
            return
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._setup_camera()
        glLightfv(GL_LIGHT0, GL_POSITION, (3.0, -4.0, 6.0, 1.0))
        self._draw_grid()
        self._draw_model(self.current_degrees, opacity=1.0, target=False)
        if self.show_target and self.target_degrees:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            self._draw_model(
                self.target_degrees,
                opacity=self.target_opacity,
                target=True,
            )
            glDisable(GL_BLEND)
        self._draw_selected_joint_guide()

    def _setup_camera(self):
        aspect = max(0.1, self.width() / max(1.0, float(self.height())))
        near, far = 0.03, 30.0
        top = near * math.tan(math.radians(45.0) / 2.0)
        right = top * aspect
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-right, right, -top, top, near, far)

        eye = self.center + self.distance * np.array((
            math.cos(self.pitch) * math.cos(self.yaw),
            math.cos(self.pitch) * math.sin(self.yaw),
            math.sin(self.pitch),
        ))
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(*eye, *self.center, 0.0, 0.0, 1.0)

    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for index in range(-12, 13):
            major = index == 0
            shade = 0.38 if major else 0.19
            glColor4f(shade, shade + 0.02, shade + 0.04, 1.0)
            coordinate = index * 0.2
            glVertex3f(coordinate, -2.4, 0.0)
            glVertex3f(coordinate, 2.4, 0.0)
            glVertex3f(-2.4, coordinate, 0.0)
            glVertex3f(2.4, coordinate, 0.0)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_model(self, joint_degrees, opacity, target, picking=False):
        transforms = self.model.link_transforms(joint_degrees)
        for index, visual in enumerate(self.model.visuals):
            if visual.link not in transforms:
                continue
            vertices, normals = self.meshes.get(
                str(visual.mesh_path), (None, None)
            )
            if vertices is None:
                continue
            scale = np.eye(4, dtype=np.float64)
            scale[0, 0], scale[1, 1], scale[2, 2] = visual.scale
            matrix = transforms[visual.link] @ visual.origin @ scale
            glPushMatrix()
            glMultMatrixf(np.ascontiguousarray(matrix.T, dtype=np.float32))
            if picking:
                identifier = index + 1
                glColor3ub(
                    identifier & 0xff,
                    (identifier >> 8) & 0xff,
                    (identifier >> 16) & 0xff,
                )
            elif target:
                glColor4f(0.12, 0.72, 1.0, opacity)
            elif (
                self.selected_joint
                and self.model.selectable_joint_for_link(visual.link)
                == self.selected_joint
            ):
                glColor4f(1.0, 0.56, 0.12, 1.0)
            else:
                red, green, blue, alpha = visual.color
                glColor4f(red, green, blue, alpha * opacity)
            glEnableClientState(GL_VERTEX_ARRAY)
            glVertexPointer(3, GL_FLOAT, 0, vertices)
            if not picking:
                glEnableClientState(GL_NORMAL_ARRAY)
                glNormalPointer(GL_FLOAT, 0, normals)
            glDrawArrays(GL_TRIANGLES, 0, len(vertices))
            if not picking:
                glDisableClientState(GL_NORMAL_ARRAY)
            glDisableClientState(GL_VERTEX_ARRAY)
            glPopMatrix()

    @staticmethod
    def _draw_line_strip(points):
        glBegin(GL_LINES)
        for start, end in zip(points[:-1], points[1:]):
            glVertex3f(*start)
            glVertex3f(*end)
        glEnd()

    @staticmethod
    def _draw_line_pairs(pairs):
        glBegin(GL_LINES)
        for start, end in pairs:
            glVertex3f(*start)
            glVertex3f(*end)
        glEnd()

    def _draw_selected_joint_guide(self):
        if not self.selected_joint:
            return
        world_axis = self.model.joint_world_axis(
            self.selected_joint,
            self.current_degrees,
        )
        if world_axis is None:
            return
        geometry = _rotation_guide_geometry(*world_axis)
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glLineWidth(3.0)

        # Purple line is the physical rotation axis.
        glColor4f(0.77, 0.52, 1.0, 1.0)
        self._draw_line_pairs((geometry['axis'],))

        # Cyan follows the URDF right-hand-rule positive direction.
        glColor4f(0.20, 0.86, 1.0, 1.0)
        self._draw_line_strip(geometry['positive_arc'])
        self._draw_line_pairs(geometry['positive_arrowheads'])

        # Orange is the negative direction.
        glColor4f(1.0, 0.55, 0.20, 1.0)
        self._draw_line_strip(geometry['negative_arc'])
        self._draw_line_pairs(geometry['negative_arrowheads'])

        glLineWidth(1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def set_joint_states(self, current_degrees, target_degrees):
        # Gripper feedback is motor rotation, not the URDF tool joint angle.
        self.current_degrees = dict(current_degrees)
        self.target_degrees = dict(target_degrees)
        self.current_degrees['Joint_Left_Gripper'] = 0.0
        self.current_degrees['Joint_Right_Gripper'] = 0.0
        self.target_degrees['Joint_Left_Gripper'] = 0.0
        self.target_degrees['Joint_Right_Gripper'] = 0.0
        self.update()

    def set_target_joint(self, name, value):
        self.target_degrees[name] = float(value)
        self.update()

    def set_selected_joint(self, name):
        if name and (name not in JOINT_LIMITS or name in GRIPPER_JOINTS):
            return
        self.selected_joint = str(name)
        self.update()

    def set_interaction_mode(self, mode):
        self.interaction_mode = 'joint' if mode == 'joint' else 'camera'
        self.setCursor(
            QtCore.Qt.CrossCursor
            if self.interaction_mode == 'joint'
            else QtCore.Qt.OpenHandCursor
        )

    def pick_joint(self, position):
        if self.render_error or not self.isValid():
            return ''
        self.makeCurrent()
        try:
            glBindFramebuffer(GL_FRAMEBUFFER, self.defaultFramebufferObject())
            ratio = self.devicePixelRatioF()
            glViewport(
                0,
                0,
                max(1, round(self.width() * ratio)),
                max(1, round(self.height() * ratio)),
            )
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self._setup_camera()
            glDisable(GL_LIGHTING)
            glDisable(GL_BLEND)
            self._draw_model(
                self.current_degrees, opacity=1.0, target=False, picking=True
            )
            pixel = glReadPixels(
                round(position.x() * ratio),
                round((self.height() - position.y() - 1) * ratio),
                1,
                1,
                GL_RGB,
                GL_UNSIGNED_BYTE,
            )
            channels = np.frombuffer(pixel, dtype=np.uint8).reshape(-1)
            if len(channels) < 3:
                return ''
            identifier = (
                int(channels[0])
                | (int(channels[1]) << 8)
                | (int(channels[2]) << 16)
            )
            if not 1 <= identifier <= len(self.model.visuals):
                return ''
            link = self.model.visuals[identifier - 1].link
            return self.model.selectable_joint_for_link(link) or ''
        except Exception:
            return ''
        finally:
            self.doneCurrent()
            self.update()

    def reset_camera(self):
        self.yaw = math.radians(42.0)
        self.pitch = math.radians(18.0)
        self.distance = 2.75
        self.center[:] = (0.0, 0.0, 0.72)
        self.update()

    def front_camera(self):
        self.yaw = 0.0
        self.pitch = math.radians(8.0)
        self.distance = 2.75
        self.update()

    def side_camera(self):
        self.yaw = math.radians(90.0)
        self.pitch = math.radians(8.0)
        self.distance = 2.75
        self.update()

    def mousePressEvent(self, event):
        if (
            self.interaction_mode == 'joint'
            and event.button() == QtCore.Qt.LeftButton
            and not (event.modifiers() & QtCore.Qt.AltModifier)
        ):
            joint = self.pick_joint(event.pos())
            if joint:
                self.set_selected_joint(joint)
                self.joint_selected.emit(joint)
                self._joint_drag_start_x = event.x()
                self._joint_drag_start_value = float(
                    self.target_degrees.get(
                        joint, self.current_degrees.get(joint, 0.0)
                    )
                )
                return
            # Empty-space drag remains available for camera orbit while the
            # operator stays in joint mode.
            self._last_mouse = event.pos()
            self._camera_drag_kind = 'orbit'
            self.setCursor(QtCore.Qt.ClosedHandCursor)
            return
        if event.button() == QtCore.Qt.LeftButton:
            self._last_mouse = event.pos()
            self._camera_drag_kind = 'orbit'
            self.setCursor(QtCore.Qt.ClosedHandCursor)
        elif event.button() in (QtCore.Qt.RightButton, QtCore.Qt.MiddleButton):
            self._last_mouse = event.pos()
            self._camera_drag_kind = 'pan'
            self.setCursor(QtCore.Qt.SizeAllCursor)

    def mouseMoveEvent(self, event):
        if (
            self.interaction_mode == 'joint'
            and event.buttons() & QtCore.Qt.LeftButton
            and self.selected_joint
            and self._joint_drag_start_x is not None
        ):
            lower, upper = JOINT_LIMITS[self.selected_joint]
            sensitivity = min(0.4, max(0.08, (upper - lower) / 650.0))
            value = self._joint_drag_start_value + (
                event.x() - self._joint_drag_start_x
            ) * sensitivity
            value = min(upper, max(lower, value))
            self.set_target_joint(self.selected_joint, value)
            self.joint_target_dragged.emit(self.selected_joint, value)
            return
        if self._last_mouse is None:
            return
        delta = event.pos() - self._last_mouse
        self._last_mouse = event.pos()
        if self._camera_drag_kind == 'orbit':
            self.yaw -= delta.x() * 0.008
            self.pitch = min(
                math.radians(85.0),
                max(math.radians(-10.0), self.pitch + delta.y() * 0.006),
            )
        elif self._camera_drag_kind == 'pan':
            scale = self.distance * 0.0015
            forward = np.array((
                -math.cos(self.pitch) * math.cos(self.yaw),
                -math.cos(self.pitch) * math.sin(self.yaw),
                -math.sin(self.pitch),
            ))
            right = np.cross(forward, np.array((0.0, 0.0, 1.0)))
            right /= max(1e-12, np.linalg.norm(right))
            camera_up = np.cross(right, forward)
            self.center += (
                -delta.x() * right + delta.y() * camera_up
            ) * scale
        self.update()

    def mouseReleaseEvent(self, _event):
        self._last_mouse = None
        self._camera_drag_kind = None
        self._joint_drag_start_x = None
        self._joint_drag_start_value = None
        self.setCursor(
            QtCore.Qt.CrossCursor
            if self.interaction_mode == 'joint'
            else QtCore.Qt.OpenHandCursor
        )

    def wheelEvent(self, event):
        steps = event.angleDelta().y() / 120.0
        self.distance = min(8.0, max(0.5, self.distance * (0.88 ** steps)))
        self.update()

    def mouseDoubleClickEvent(self, _event):
        self.reset_camera()


class NoWheelViewSlider(QtWidgets.QSlider):
    def wheelEvent(self, event):
        event.ignore()


class RobotViewPanel(QtWidgets.QFrame):
    joint_selected = QtCore.pyqtSignal(str)
    joint_target_changed = QtCore.pyqtSignal(str, float)

    def __init__(self, urdf_path, parent=None):
        super().__init__(parent)
        self.setObjectName('robotViewPanel')
        self._action_preview_active = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(7)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('机器人实时三维视图')
        title.setObjectName('viewTitle')
        self.state_dot = QtWidgets.QLabel('●')
        self.state_dot.setObjectName('connectionOff')
        self.state_label = QtWidgets.QLabel('等待关节反馈')
        self.target_toggle = QtWidgets.QCheckBox('显示目标虚影')
        self.target_toggle.setChecked(True)
        reset_button = QtWidgets.QPushButton('复位视角')
        front_button = QtWidgets.QPushButton('正视')
        side_button = QtWidgets.QPushButton('侧视')
        self.camera_mode_button = QtWidgets.QToolButton()
        self.camera_mode_button.setText('观察')
        self.camera_mode_button.setCheckable(True)
        self.joint_mode_button = QtWidgets.QToolButton()
        self.joint_mode_button.setText('调关节')
        self.joint_mode_button.setCheckable(True)
        mode_group = QtWidgets.QButtonGroup(self)
        mode_group.setExclusive(True)
        mode_group.addButton(self.camera_mode_button)
        mode_group.addButton(self.joint_mode_button)
        self.camera_mode_button.setChecked(True)
        header.addWidget(title)
        header.addWidget(self.state_dot)
        header.addWidget(self.state_label)
        header.addStretch()
        header.addWidget(self.camera_mode_button)
        header.addWidget(self.joint_mode_button)
        header.addWidget(self.target_toggle)
        header.addWidget(front_button)
        header.addWidget(side_button)
        header.addWidget(reset_button)
        layout.addLayout(header)

        self.view = RobotOpenGLWidget(urdf_path)
        layout.addWidget(self.view, 1)
        self.hint = QtWidgets.QLabel(
            '左键拖动旋转 · 右键拖动平移 · 滚轮缩放 · 双击复位'
        )
        self.hint.setObjectName('safetyHint')
        self.hint.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.hint)

        self.adjust_card = QtWidgets.QFrame()
        self.adjust_card.setObjectName('jointAdjustCard')
        adjust = QtWidgets.QHBoxLayout(self.adjust_card)
        adjust.setContentsMargins(10, 7, 10, 7)
        self.selected_label = QtWidgets.QLabel('请选择机器人关节')
        self.selected_label.setObjectName('selectedJointLabel')
        self.selected_value = QtWidgets.QLabel('当前 --.-°')
        self.direction_guide = QtWidgets.QLabel(
            '<span style="color:#ff8f3d">− ↶</span> '
            '左拖减小　右拖增大 '
            '<span style="color:#35d5ff">↷ +</span>'
        )
        self.direction_guide.setObjectName('jointDirectionGuide')
        self.direction_guide.setTextFormat(QtCore.Qt.RichText)
        self.direction_guide.setToolTip(
            '模型中的紫线是旋转轴；橙色箭头为负方向，青色箭头为正方向'
        )
        self.direction_guide.setVisible(False)
        self.selected_slider = NoWheelViewSlider(QtCore.Qt.Horizontal)
        self.selected_slider.setEnabled(False)
        self.selected_box = QtWidgets.QDoubleSpinBox()
        self.selected_box.setDecimals(1)
        self.selected_box.setSingleStep(0.5)
        self.selected_box.setSuffix('°')
        self.selected_box.setMinimumWidth(88)
        self.selected_box.setEnabled(False)
        self.sync_button = QtWidgets.QPushButton('同步当前')
        self.sync_button.setEnabled(False)
        adjust.addWidget(self.selected_label)
        adjust.addWidget(self.selected_value)
        adjust.addWidget(self.direction_guide)
        adjust.addWidget(self.selected_slider, 1)
        adjust.addWidget(self.selected_box)
        adjust.addWidget(self.sync_button)
        layout.addWidget(self.adjust_card)
        self._card_changing = False

        self.target_toggle.toggled.connect(self._target_toggled)
        reset_button.clicked.connect(self.view.reset_camera)
        front_button.clicked.connect(self.view.front_camera)
        side_button.clicked.connect(self.view.side_camera)
        self.camera_mode_button.toggled.connect(self._mode_changed)
        self.view.joint_selected.connect(self._joint_selected)
        self.view.joint_target_dragged.connect(self._view_target_changed)
        self.selected_slider.valueChanged.connect(self._slider_changed)
        self.selected_box.valueChanged.connect(self._box_changed)
        self.sync_button.clicked.connect(self._sync_selected)

    def _target_toggled(self, checked):
        self.view.show_target = bool(checked)
        self.view.update()

    def set_joint_states(self, current_degrees, target_degrees):
        self.view.set_joint_states(current_degrees, target_degrees)
        self._update_card_values()

    def set_action_preview_active(self, active):
        """Make action playback unmistakable without affecting robot control."""
        active = bool(active)
        self._action_preview_active = active
        self.target_toggle.setChecked(True)
        self.view.show_target = True
        self.view.target_opacity = 0.62 if active else 0.28
        self.view.update()
        self.state_label.setText(
            '蓝色虚影播放中' if active
            else ('实时更新' if self.view.connected else '等待关节反馈')
        )

    def set_connected(self, connected):
        self.view.connected = bool(connected)
        self.state_dot.setObjectName(
            'connectionOn' if connected else 'connectionOff'
        )
        self.state_dot.style().unpolish(self.state_dot)
        self.state_dot.style().polish(self.state_dot)
        self.state_label.setText(
            '蓝色虚影播放中'
            if self._action_preview_active
            else ('实时更新' if connected else '等待关节反馈')
        )

    def _mode_changed(self, camera_checked):
        joint_mode = not bool(camera_checked)
        self.view.set_interaction_mode('joint' if joint_mode else 'camera')
        self.hint.setText(
            '点击关节选择 · 拖动关节左右调节 · 拖动空白处旋转视角 · '
            '右键平移 · 滚轮缩放'
            if joint_mode
            else '左键拖动旋转 · 右键拖动平移 · 滚轮缩放 · 双击复位'
        )

    def set_joint_mode(self):
        self.joint_mode_button.setChecked(True)

    def select_joint(self, name):
        if name not in JOINT_LIMITS or name in GRIPPER_JOINTS:
            return
        self.view.set_selected_joint(name)
        lower, upper = JOINT_LIMITS[name]
        self._card_changing = True
        self.selected_slider.setRange(round(lower * 10), round(upper * 10))
        self.selected_box.setRange(lower, upper)
        self._card_changing = False
        self.selected_slider.setEnabled(True)
        self.selected_box.setEnabled(True)
        self.sync_button.setEnabled(True)
        self.direction_guide.setVisible(True)
        self.selected_label.setText(JOINT_LABELS.get(name, name))
        self.selected_label.setToolTip(name)
        self._update_card_values()

    def _joint_selected(self, name):
        self.select_joint(name)
        self.joint_selected.emit(name)

    def _update_card_values(self):
        name = self.view.selected_joint
        if not name:
            return
        current = self.view.current_degrees.get(name)
        target = self.view.target_degrees.get(name, current)
        self.selected_value.setText(
            '当前 --.-°' if current is None else f'当前 {current:.1f}°'
        )
        if target is None:
            return
        self._card_changing = True
        self.selected_slider.setValue(round(float(target) * 10))
        self.selected_box.setValue(float(target))
        self._card_changing = False

    def _emit_target(self, value):
        name = self.view.selected_joint
        if not name:
            return
        self.view.set_target_joint(name, float(value))
        self.joint_target_changed.emit(name, float(value))

    def _slider_changed(self, value):
        if self._card_changing:
            return
        target = value / 10.0
        self._card_changing = True
        self.selected_box.setValue(target)
        self._card_changing = False
        self._emit_target(target)

    def _box_changed(self, value):
        if self._card_changing:
            return
        self._card_changing = True
        self.selected_slider.setValue(round(float(value) * 10))
        self._card_changing = False
        self._emit_target(value)

    def _view_target_changed(self, name, value):
        self._update_card_values()
        self.joint_target_changed.emit(name, value)

    def _sync_selected(self):
        name = self.view.selected_joint
        current = self.view.current_degrees.get(name)
        if current is not None:
            self._emit_target(current)
