"""Numerical contract against BeamNG 0.39.4's passive quaternion convention.

This reads the production Lua expressions, but does not execute Lua or the
engine setter. The reference algebra comes from the locally shipped mathlib:
LuaQuat.__mul (lines 966-980), inversed (957-960), and toDirUp (1150-1152).
techCore's handleTeleportVehicle (910-912) supplies the relative-setter rule.
See beamng_math_source_20261006.json for the source paths and SHA256 digests.
"""
import ast
import math
from pathlib import Path
import re
import unittest


SOURCE = Path(__file__).parents[1] / 'tools/beamng_extensions/wildosKinematic.lua'


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


class Quat:
    def __init__(self, x, y, z, w):
        self.values = (x, y, z, w)

    def __mul__(self, other):
        x, y, z, w = self.values
        X, Y, Z, W = other.values
        return Quat(w*X+x*W+y*Z-z*Y, w*Y+y*W+z*X-x*Z,
                    w*Z+z*W+x*Y-y*X, w*W-x*X-y*Y-z*Z)

    def inversed(self):
        x, y, z, w = self.values
        n = sum(v*v for v in self.values)
        return Quat(-x/n, -y/n, -z/n, w/n)

    def rotate(self, vector):
        # BeamNG: v - 2*w*cross(q.xyz,v) + cross(q.xyz,2*cross(q.xyz,v)).
        xyz, w = self.values[:3], self.values[3]
        qv = tuple(2*v for v in cross(xyz, vector))
        second = cross(xyz, qv)
        return tuple(v-w*c+d for v, c, d in zip(vector, qv, second))


def axis_angle(axis, angle):
    s = math.sin(angle/2)
    return Quat(*(s*v for v in axis), math.cos(angle/2))


def rz(vector, angle):
    x, y, z = vector
    return (x*math.cos(angle)-y*math.sin(angle),
            x*math.sin(angle)+y*math.cos(angle), z)


def source_expression(expression, values):
    """Evaluate only the small arithmetic subset used by the selected Lua lines."""
    parsed = ast.parse(expression, mode='eval')
    allowed = (ast.Expression, ast.BinOp, ast.Mult, ast.Add, ast.Sub, ast.Div,
               ast.UnaryOp, ast.USub, ast.Constant, ast.Name, ast.Load, ast.Call,
               ast.Attribute)
    for node in ast.walk(parsed):
        if not isinstance(node, allowed):
            raise AssertionError(f'Unsupported Lua arithmetic: {expression}')
        if isinstance(node, ast.Name) and node.id not in values:
            raise AssertionError(f'Unknown arithmetic name: {node.id}')
        if isinstance(node, ast.Attribute) and not (
                isinstance(node.value, ast.Name) and node.value.id == 'math'
                and node.attr in ('cos', 'sin')):
            raise AssertionError('Only math.sin/cos attributes are supported')
    return eval(compile(parsed, '<selected Lua arithmetic>', 'eval'), {'__builtins__': {}}, values)


class BeamNGKinematicMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding='utf8')
        cls.target = re.search(r'^\s*local target = (.+)$', cls.source, re.M).group(1)
        cls.sign = int(re.search(r'local rosYawToClusterSign = (-?\d+)', cls.source).group(1))

    def assert_vector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=11)

    def target_from_source(self, current, turn):
        return source_expression(self.target, dict(current=current, turn=turn,
            rosYawToClusterSign=self.sign, quatFromAxisAngle=axis_angle,
            vec3=lambda *args: args))

    def orientations(self):
        # Include level/yaw cases (which conceal the multiplication-order bug)
        # and independent pitch/roll; no Town-specific angles or terrain rules.
        for yaw in (0., 1.4, -2.9):
            for pitch, roll in ((0., 0.), (.3, -.2), (-.45, .5)):
                yield (axis_angle((1, 0, 0), roll)
                       * axis_angle((0, 1, 0), pitch) * axis_angle((0, 0, 1), yaw))

    def test_axis_angle_has_engine_passive_sign(self):
        self.assert_vector(axis_angle((0, 0, 1), .7).rotate((1, 0, 0)),
                           rz((1, 0, 0), -.7))

    def test_current_lua_rotates_all_axes_about_world_vertical(self):
        for current in self.orientations():
            for turn in (-.6, -.02, .02, .6):
                with self.subTest(current=current.values, turn=turn):
                    target = self.target_from_source(current, turn)
                    for axis in ((0, 1, 0), (0, 0, 1), (1, 0, 0)):
                        self.assert_vector(target.rotate(axis), rz(current.rotate(axis), turn))

    def test_relative_argument_reconstructs_absolute_target(self):
        self.assertIn('local delta = current:inversed()*target', self.source)
        for current in self.orientations():
            target = self.target_from_source(current, .4)
            delta = current.inversed()*target
            actual = current*delta  # Absolute-to-relative rule used by techCore.
            self.assert_vector(actual.rotate((0, 1, 0)), target.rotate((0, 1, 0)))

    def test_rotation_preserves_navigation_origin_with_tilt(self):
        for current in self.orientations():
            f = current.rotate((0, 1, 0))
            left = cross(current.rotate((0, 0, 1)), f)
            for turn in (-.4, .4):
                ox, oy, oz = tuple(-2.172*a+.330*b for a, b in zip(f, left))
                values = dict(ox=ox, oy=oy, turn=turn, math=math)
                projected = []
                for name in ('nextOx', 'nextOy'):
                    expression = re.search(rf'^\s*local {name} = (.+)$', self.source, re.M).group(1)
                    projected.append(source_expression(expression, values))
                target = self.target_from_source(current, turn)
                nf = target.rotate((0, 1, 0))
                nl = cross(target.rotate((0, 0, 1)), nf)
                new_offset = tuple(-2.172*a+.330*b for a, b in zip(nf, nl))
                # Ref-node compensation plus the *actual* rotated lever arm
                # must leave all three coordinates of the nav origin fixed.
                self.assert_vector((ox-projected[0]+new_offset[0],
                                    oy-projected[1]+new_offset[1], new_offset[2]), (ox, oy, oz))

    def test_old_world_order_fails_when_tilted(self):
        current = axis_angle((1, 0, 0), .5)*axis_angle((0, 1, 0), .3)
        wrong = axis_angle((0, 0, 1), -.4)*current
        expected = rz(current.rotate((0, 0, 1)), .4)
        self.assertGreater(math.dist(wrong.rotate((0, 0, 1)), expected), .1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
