"""Provided infrastructure tests, no student answer required."""
import json
from pathlib import Path
import numpy as np
import pytest
from nav_obstacle_map.global_prior import prior_grid, overlay_local


def spec():
    return dict(resolution=1.,width=8,height=5,origin_xy=[0.,0.],inflation_radius=0.,
                pillars=[dict(x=1.5,y=1.5,radius=.1)])


def test_prior_keeps_known_obstacle_when_local_reports_free():
    s=spec();p=prior_grid(s)
    assert p[1,1] == 100
    out=overlay_local(p,s,np.zeros((3,4),dtype=np.int8),(0.,0.),1.)
    assert out[1,1] == 100
    assert np.array_equal(p,prior_grid(s))


def test_latest_scan_adds_then_removes_only_observed_obstacles():
    s=spec();p=prior_grid(s);local=np.zeros((2,3),dtype=np.int8);local[1,1]=100
    first=overlay_local(p,s,local,(3.,1.),1.)
    assert first[2,4] == 100 and p[2,4] == 0
    second=overlay_local(p,s,np.zeros_like(local),(3.,1.),1.)
    assert second[2,4] == 0 and second[1,1] == 100


def test_rolling_window_clips_at_global_edges():
    s=spec();p=prior_grid(s)
    out=overlay_local(p,s,np.full((2,3),100,dtype=np.int8),(-1.,4.),1.)
    assert (out[4,:2] == 100).all()
    assert np.array_equal(out[:4],p[:4])


def test_alignment_and_resolution_rejected():
    s=spec();p=prior_grid(s)
    for origin,resolution in [((.3,0.),1.),((0.,0.),.5)]:
        with pytest.raises(ValueError):overlay_local(p,s,np.zeros((2,2)),origin,resolution)


def test_half_prior_has_exactly_sixty_unique_selected_pillars():
    path=Path(__file__).parents[1]/'resource/forest-half-prior.json'
    s=json.loads(path.read_text())
    assert s['source_pillar_count'] == 120
    assert len(s['pillars']) == len(set(s['selected_pillar_indices'])) == 60
    assert np.count_nonzero(prior_grid(s)) > 0
