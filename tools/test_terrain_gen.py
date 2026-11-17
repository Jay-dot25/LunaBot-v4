#!/usr/bin/env python3
"""
Test generator for the Canonical Lunar Terrain specification.
"""
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

EXTENT = 400.0
N_FINE = 320
N_COLL = 100
MISSION_COLL_STEP = 1.0
TARGET_H0 = -2.788
BND_IN = 170.0
BND_OUT = 200.0

def value_noise(rng, n, extent, wavelength):
    m = max(2, int(round(extent / wavelength)))
    g = rng.random((m + 1, m + 1))
    u = np.linspace(0.0, m, n)
    U, V = np.meshgrid(u, u, indexing="xy")
    ix = np.clip(np.floor(U).astype(int), 0, m - 1)
    jx = np.clip(np.floor(V).astype(int), 0, m - 1)
    fx = U - ix
    fx = fx * fx * (3.0 - 2.0 * fx)
    fy = V - jx
    fy = fy * fy * (3.0 - 2.0 * fy)
    a = g[jx, ix]
    b = g[jx, ix + 1]
    c = g[jx + 1, ix]
    d = g[jx + 1, ix + 1]
    ab = a + (b - a) * fx
    cd = c + (d - c) * fx
    return (ab + (cd - ab) * fy) * 2.0 - 1.0

def dist_segment(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx*dx + dy*dy
    if l2 == 0:
        return np.sqrt((px - x1)**2 + (py - y1)**2)
    t = np.clip(((px - x1)*dx + (py - y1)*dy) / l2, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return np.sqrt((px - proj_x)**2 + (py - proj_y)**2)

def dist_polyline(px, py, points):
    d_min = np.full_like(px, 1e9)
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i+1]
        d = dist_segment(px, py, x1, y1, x2, y2)
        d_min = np.minimum(d_min, d)
    return d_min

print("Helper functions compiled cleanly.")
