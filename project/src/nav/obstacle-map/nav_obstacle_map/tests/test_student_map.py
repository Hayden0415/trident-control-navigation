"""Public acceptance tests. Expected to FAIL until both TODOs are implemented."""
import numpy as np
from nav_obstacle_map.student_map import GridSpec, project_cloud, inflate_grid


def spec(**kwargs):
    return GridSpec(resolution=1.0, width=5, height=4, origin_x=-2.0, origin_y=-1.0,
                    z_min=0.2, z_max=1.0, **kwargs)


def test_projection_height_boundaries_origin_and_duplicates():
    p = np.array([[-1.9, -0.9, 0.2], [-1.1, -0.1, 1.0], [0.1, 1.1, 0.6],
                  [0.1, 1.1, 0.6], [-2.01, 0, 0.5], [3.0, 0, 0.5], [0, 3.0, 0.5],
                  [1, 1, 0.19], [1, 1, 1.01], [np.nan, 0, 0.5], [0, np.inf, 0.5],
                  [0, 0, np.nan]])
    g = project_cloud(p, spec())
    expected = np.zeros((4, 5), dtype=np.uint8)
    expected[0, 0] = expected[2, 2] = 100
    np.testing.assert_array_equal(g, expected)
    assert g.dtype == np.uint8


def test_empty_snapshot_and_no_history():
    s = spec()
    project_cloud(np.array([[0, 0, 0.5]]), s)
    np.testing.assert_array_equal(project_cloud(np.empty((0, 3)), s), np.zeros((4, 5), np.uint8))


def test_disk_inflation_no_cascade_and_no_mutation():
    s = spec(inflation_radius=1.0)
    raw = np.zeros((4, 5), dtype=np.uint8)
    raw[1, 2] = 100
    saved = raw.copy()
    g = inflate_grid(raw, s)
    expected = raw.copy()
    expected[0, 2] = expected[2, 2] = expected[1, 1] = expected[1, 3] = 100
    np.testing.assert_array_equal(g, expected)
    np.testing.assert_array_equal(raw, saved)
    assert not np.shares_memory(raw, g)
    assert g.dtype == np.uint8


def test_corner_and_fractional_radius():
    raw = np.zeros((4, 5), dtype=np.uint8)
    raw[0, 0] = 100
    g = inflate_grid(raw, spec(inflation_radius=1.42))
    expected = raw.copy()
    expected[0, 1] = expected[1, 0] = expected[1, 1] = 100
    np.testing.assert_array_equal(g, expected)
    np.testing.assert_array_equal(inflate_grid(raw, spec(inflation_radius=0)), raw)
    np.testing.assert_array_equal(inflate_grid(raw, spec(inflation_radius=0.9)), raw)


def test_non_square_and_multiple_seeds():
    raw = np.zeros((4, 5), dtype=np.uint8)
    raw[0, 0] = raw[3, 4] = 100
    expected = raw.copy()
    expected[0, 1] = expected[1, 0] = expected[2, 4] = expected[3, 3] = 100
    np.testing.assert_array_equal(inflate_grid(raw, spec(inflation_radius=1)), expected)
