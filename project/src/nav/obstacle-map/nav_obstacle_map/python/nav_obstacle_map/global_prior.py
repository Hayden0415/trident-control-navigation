"""Provided static prior and per-scan obstacle overlay; independent of student TODOs."""
import math
import numpy as np


def prior_grid(spec):
    resolution = float(spec['resolution'])
    width, height = int(spec['width']), int(spec['height'])
    ox, oy = map(float, spec['origin_xy'])
    margin = float(spec['inflation_radius'])
    if (not all(math.isfinite(v) for v in (resolution, ox, oy, margin))
            or resolution <= 0 or margin < 0 or width <= 0 or height <= 0
            or width*height > 250000):
        raise ValueError('invalid prior grid specification')
    # Mark cells intersecting inflated circles; conservative cell diagonal allowance.
    yy, xx = np.mgrid[:height, :width]
    wx, wy = ox+(xx+.5)*resolution, oy+(yy+.5)*resolution
    grid = np.zeros((height, width), dtype=np.uint8)
    for pillar in spec['pillars']:
        x,y,r = (float(pillar[k]) for k in ('x','y','radius'))
        if not all(math.isfinite(v) for v in (x,y,r)) or r <= 0:
            raise ValueError('invalid prior pillar')
        grid[np.hypot(wx-x, wy-y) <= r+margin+resolution/math.sqrt(2)] = 100
    return grid


def overlay_local(prior, spec, local, origin_xy, resolution):
    """Obstacle union from latest local frame only. Never erase known obstacles.

    Rolling origins must align to the same lattice. Free/unknown local values
    add no obstacle; no observed obstacle survives after leaving the next scan.
    Both layers are already inflated and are not inflated again here.
    """
    r = float(spec['resolution'])
    if not math.isclose(float(resolution), r, rel_tol=1e-6, abs_tol=1e-8):
        raise ValueError('local/prior resolution mismatch')
    offset = [(float(a)-float(b))/r for a,b in zip(origin_xy,spec['origin_xy'])]
    if not all(math.isfinite(v) and abs(v-round(v)) < 1e-4 for v in offset):
        raise ValueError('local/prior origins must share a lattice')
    if local.ndim != 2 or not np.isin(local, [-1,0,100]).all():
        raise ValueError('invalid local occupancy')
    dx,dy = map(round,offset)
    h,w = local.shape; ph,pw = prior.shape
    x0,y0,x1,y1 = max(0,dx),max(0,dy),min(pw,dx+w),min(ph,dy+h)
    result = prior.copy()
    if x1>x0 and y1>y0:
        observed=local[y0-dy:y1-dy,x0-dx:x1-dx]
        result[y0:y1,x0:x1][observed == 100] = 100
    return result
