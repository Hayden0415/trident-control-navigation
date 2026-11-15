"""学生仅需完成本文件的两个函数；接口和语义见 README。"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float = 0.1
    width: int = 100
    height: int = 80
    origin_x: float = -1.0
    origin_y: float = -1.0
    z_min: float = 0.15
    z_max: float = 1.5
    inflation_radius: float = 0.3

    def __post_init__(self):
        values = (self.resolution, self.origin_x, self.origin_y,
                  self.z_min, self.z_max, self.inflation_radius)
        if (not all(np.isfinite(v) for v in values) or self.resolution <= 0
                or not isinstance(self.width, int) or not isinstance(self.height, int)
                or self.width <= 0 or self.height <= 0
                or self.width * self.height > 250000
                or self.z_min > self.z_max or self.inflation_radius < 0):
            raise ValueError('invalid grid specification')


def project_cloud(points: np.ndarray, spec: GridSpec) -> np.ndarray:
    """TODO A：N×3 米制世界点云 -> (height,width) uint8，值为 0 或 100。

    过滤非有限坐标，保留 z_min <= z <= z_max。
    左下角原点，floor 索引；地图上、右边界不包含；越界忽略。
    同一列只要存在一个有效点，对应单元即为 100，其他为 0。
    每帧仅含当前雷达扫描；spec 是随机器人移动的局部窗口。
    每帧从零重建，不累积历史、不做衰减或射线遮挡推理。
    """
    grid = np.zeros((spec.height, spec.width), dtype=np.uint8)   # 每帧从零重建
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:                                        # 空扫描 → 全 0
        return grid

    finite = np.isfinite(pts).all(axis=1)                        # 忽略 NaN/Inf
    z_ok = (pts[:, 2] >= spec.z_min) & (pts[:, 2] <= spec.z_max)
    keep = finite & z_ok
    x, y = pts[keep, 0], pts[keep, 1]

    ix = np.floor((x - spec.origin_x) / spec.resolution).astype(np.int64)
    iy = np.floor((y - spec.origin_y) / spec.resolution).astype(np.int64)

    inside = (ix >= 0) & (ix < spec.width) & (iy >= 0) & (iy < spec.height)
    ix, iy = ix[inside], iy[inside]

    grid[iy, ix] = 100
    return grid


def inflate_grid(occupied: np.ndarray, spec: GridSpec) -> np.ndarray:
    """TODO B：返回独立副本，不修改输入，shape/dtype/二值编码不变。

    对每个原始占据格，标记中心距离 <= inflation_radius (+1e-9 米容差)
    的其他格为 100，使用欧氏圆盘，边界截断；radius=0 保持输入。
    禁止一边写输出一边把新增占据格当作膨胀种子，否则会连锁填满。
    """
    out = occupied.copy()                                        # 独立副本，不改输入
    if spec.inflation_radius <= 0:                               # 半径 0 → 内容不变
        return out

    seeds = np.argwhere(occupied == 100)                         # 原始占据格 [iy, ix]
    if seeds.shape[0] == 0:
        return out

    height, width = out.shape
    r_cells = (spec.inflation_radius + 1e-9) / spec.resolution   # 米 → 格子数
    r_int = int(np.ceil(r_cells))                                # 模板半径（格），向上取整

    # 只算一次圆盘模板（以原点为中心），避免每个种子都对全图 hypot
    dy, dx = np.mgrid[-r_int:r_int + 1, -r_int:r_int + 1]
    disk = np.hypot(dx, dy) <= r_cells

    hit = np.zeros((height, width), dtype=bool)
    for sy, sx in seeds:                                         # 每个种子：平移模板+裁剪（边界判定）+OR
        r0 = max(0, sy - r_int); r1 = min(height, sy + r_int + 1)# r0，c0对应左上角
        c0 = max(0, sx - r_int); c1 = min(width,  sx + r_int + 1)# r1，c1对应右下角
        tr0 = r0 - (sy - r_int); tr1 = tr0 + (r1 - r0)
        tc0 = c0 - (sx - r_int); tc1 = tc0 + (c1 - c0)
        hit[r0:r1, c0:c1] |= disk[tr0:tr1, tc0:tc1]
    out[hit] = 100
    return out
