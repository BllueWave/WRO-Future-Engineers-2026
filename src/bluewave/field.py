"""The WRO Future Engineers 2026 field as the robot's map.

Frame: origin at the field centre, x right, y up, millimetres; headings in radians, CCW from +x.  The same frame the
simulator (mock.World) uses.  Obstacle Challenge geometry [RULE 13.1, p.13]: exterior wall inner faces at +-1500, the
island's outer faces at +-500 -> every corridor is 1000 mm.  Walls are 100 mm tall [RULE 13.3/13.5].

Straight sections, numbered counter-clockwise: 0 = S (bottom), 1 = E, 2 = N, 3 = W.  The section the car starts in is
ALWAYS called S: the walls are 4-fold symmetric, so that is a choice of frame, not an assumption.  Facing +x in S means
the island is on the LEFT -> the round is counter-clockwise; facing -x means clockwise.

Traffic-sign seats [RULE p.7, measured off Fig.3/4/7c in webots/world8/make_world8.py]: 6 per straight, along the
section at its two ends and its middle, across it 400 and 600 mm from the OUTER wall.
The parking lot [RULE p.8, 13.25-13.29]: always in the starting section, flush against the outer wall, 200 mm deep,
1.5 x the robot's length long, between two 200 x 20 x 100 magenta limitations.
"""
from __future__ import annotations

import math

import numpy as np

HALF, ISL = 1500.0, 500.0
WALL_H = 100.0
SEAT_ALONG = (-500.0, 0.0, 500.0)          # along a section, from its centre, in CCW travel direction
SEAT_ACROSS = (400.0, 600.0)               # from the outer wall
PILLAR = 50.0                              # mm square, 100 tall
LOT_DEPTH = 200.0
LIMIT = (200.0, 20.0)                      # magenta limitation: across the lot (from the wall), along the wall


def square(h: float):
    c = [(-h, -h), (h, -h), (h, h), (-h, h)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


WALLS = np.array([[a[0], a[1], b[0], b[1]] for a, b in square(HALF) + square(ISL)], float)


def rot(k: int, x: float, y: float):
    """Rotate a point of section S into section k (k * 90 deg CCW)."""
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    return c * x - s * y, s * x + c * y


def section_heading(k: int, direction: int) -> float:
    """Travel heading in straight k: +1 = counter-clockwise round, -1 = clockwise."""
    return (k * math.pi / 2 + (0.0 if direction > 0 else math.pi) + math.pi) % (2 * math.pi) - math.pi


def lane_point(k: int, along: float, d_outer: float, direction: int):
    """The field point `along` mm past the centre of straight k (in TRAVEL direction) and `d_outer` mm off its outer
    wall."""
    return rot(k, direction * along, -HALF + d_outer)


def seats():
    """[(section, along_index, across_index, x, y)] -- the 24 legal sign positions."""
    out = []
    for k in range(4):
        for ai, a in enumerate(SEAT_ALONG):
            for ci, d in enumerate(SEAT_ACROSS):
                x, y = rot(k, a, -HALF + d)
                out.append((k, ai, ci, x, y))
    return out


SEATS = seats()
SEAT_XY = np.array([(s[3], s[4]) for s in SEATS])


def where(x: float, y: float):
    """('straight', k) or ('corner', k) where corner k joins straight k to straight k+1 (CCW), or ('out', -1)."""
    ax, ay = abs(x), abs(y)
    if ax > HALF or ay > HALF or (ax < ISL and ay < ISL):
        return "out", -1
    if ay > ISL and ax <= ISL:
        return "straight", 0 if y < 0 else 2
    if ax > ISL and ay <= ISL:
        return "straight", 1 if x > 0 else 3
    return "corner", {(1, -1): 0, (1, 1): 1, (-1, 1): 2, (-1, -1): 3}[(1 if x > 0 else -1, 1 if y > 0 else -1)]


def progress(x: float, y: float, direction: int) -> float:
    """Continuous lap coordinate in quarter turns, 0 at the centre of S, growing in the travel direction.
    One lap = 4.0.  From the angle around the field centre, which is monotone along any sane path."""
    a = math.atan2(y, x) + math.pi / 2                 # 0 at the bottom (S centre), CCW +
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return direction * a / (math.pi / 2)


class DistField:
    """Distance (mm) from any field point to the nearest wall face, on a grid.  The localiser's likelihood field."""

    def __init__(self, walls: np.ndarray = WALLS, res: float = 10.0, lim: float = HALF + 250.0):
        self.res, self.lim = res, lim
        n = int(2 * lim / res) + 1
        g = -lim + np.arange(n) * res
        X, Y = np.meshgrid(g, g)                       # [iy, ix]
        d = np.full(X.shape, np.inf)
        for x1, y1, x2, y2 in walls:
            ex, ey = x2 - x1, y2 - y1
            t = np.clip(((X - x1) * ex + (Y - y1) * ey) / (ex * ex + ey * ey), 0, 1)
            d = np.minimum(d, np.hypot(X - (x1 + t * ex), Y - (y1 + t * ey)))
        self.d = d.astype(np.float32)
        self.n = n

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        ix = np.clip(np.rint((x + self.lim) / self.res).astype(np.int32), 0, self.n - 1)
        iy = np.clip(np.rint((y + self.lim) / self.res).astype(np.int32), 0, self.n - 1)
        out = self.d[iy, ix]
        far = (np.abs(x) > self.lim) | (np.abs(y) > self.lim)
        return np.where(far, 999.0, out)


def raycast(x: float, y: float, ang: np.ndarray, walls: np.ndarray = WALLS, max_mm: float = 5000.0) -> np.ndarray:
    """Distance along each ray (field angles) to the first wall face."""
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    x1, y1, x2, y2 = walls[:, 0], walls[:, 1], walls[:, 2], walls[:, 3]
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
    t = np.where((den != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    return np.minimum(t.min(axis=1), max_mm)


def to_section(k: int, x: float, y: float):
    """Field point -> (along from straight k's centre in the CCW sense, distance from k's outer wall)."""
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    a, b = c * x + s * y, -s * x + c * y                   # rotate back by -k * 90 deg
    return a, b + HALF


# ---------------------------------------------------------------------------------- BRAIN4_SPEC 4.2: layouts, symmetry
def island_of(corr=(1000.0, 1000.0, 1000.0, 1000.0)):
    """The island's outer faces (left, right, bottom, top) for corridor widths (S, E, N, W) -- exactly as mock.World
    builds it from a scene's `corridors`: (-1500 + W, 1500 - E, -1500 + S, 1500 - N).  The standard field = 1000 x 4."""
    cS, cE, cN, cW = (float(v) for v in corr)
    return (-HALF + cW, HALF - cE, -HALF + cS, HALF - cN)


def layout_walls(corr=(1000.0, 1000.0, 1000.0, 1000.0)) -> np.ndarray:
    """Wall faces (8 x 4: x1, y1, x2, y2) of a field whose corridors are (S, E, N, W) wide: the outer square and the
    island of island_of().  layout_walls() == WALLS (the standard field) up to the island's corner order."""
    l, r, b, t = island_of(corr)
    isl = [((l, b), (r, b)), ((r, b), (r, t)), ((r, t), (l, t)), ((l, t), (l, b))]
    return np.array([[a[0], a[1], c[0], c[1]] for a, c in square(HALF) + isl], float)


def rot_pose(pose, k: int):
    """R_k: the pose (x, y, th) rotated by k x 90 deg CCW about the field centre (BRAIN4_SPEC 2 "Field symmetry")."""
    x, y = rot(k, float(pose[0]), float(pose[1]))
    th = (float(pose[2]) + (k % 4) * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
    return x, y, th


def rot_seat(j: int, k: int) -> int:
    """The seat index (SEATS order) that seat j becomes under R_k: (s, along, row) -> ((s + k) mod 4, along, row)."""
    s, ai, ci = SEATS[j][:3]
    return ((s + k) % 4) * 6 + ai * 2 + ci


def section_of_seat(j: int) -> int:
    """The straight (0 S, 1 E, 2 N, 3 W) seat j stands in."""
    return int(SEATS[j][0])
