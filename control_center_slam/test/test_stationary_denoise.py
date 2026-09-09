import math
from sensor_msgs.msg import LaserScan
from mapping_scan_adapter import StationaryScanDenoiser

def scan(t, values):
    m=LaserScan();m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
    m.header.frame_id='laser';m.angle_increment=.01;m.range_min=.04;m.range_max=16.
    m.ranges=[float(x) for x in values];return m

def test_median_only_when_stationary():
    f=StationaryScanDenoiser()
    f.filter(scan(1,[1]),True);f.filter(scan(1.1,[1.01]),True)
    assert abs(f.filter(scan(1.2,[1.05]),True).ranges[0]-1.01)<1e-5

def test_motion_bypasses_and_clears_history():
    f=StationaryScanDenoiser()
    f.filter(scan(1,[1]),True);f.filter(scan(1.1,[1]),True)
    assert abs(f.filter(scan(1.2,[1.07]),False).ranges[0]-1.07)<1e-5
    assert abs(f.filter(scan(1.3,[1.06]),True).ranges[0]-1.06)<1e-5

def test_new_close_obstacle_has_no_delay():
    f=StationaryScanDenoiser()
    f.filter(scan(1,[3]),True);f.filter(scan(1.1,[3]),True)
    assert abs(f.filter(scan(1.2,[.5]),True).ranges[0]-.5)<1e-5

def test_invalid_rays_are_never_filled():
    f=StationaryScanDenoiser()
    for t in [1,1.1]:f.filter(scan(t,[1,1]),True)
    m=f.filter(scan(1.2,[float('nan'),float('inf')]),True)
    assert math.isnan(m.ranges[0]) and math.isinf(m.ranges[1])

def test_gap_resets_history():
    f=StationaryScanDenoiser();f.filter(scan(1,[1]),True);f.filter(scan(1.1,[1]),True)
    assert abs(f.filter(scan(1.5,[1.07]),True).ranges[0]-1.07)<1e-5

def test_geometry_resets_history():
    f=StationaryScanDenoiser();f.filter(scan(1,[1]),True);f.filter(scan(1.1,[1]),True)
    m=scan(1.2,[1.07]);m.header.frame_id='other'
    assert abs(f.filter(m,True).ranges[0]-1.07)<1e-5

def test_no_spatial_smoothing_of_thin_leg():
    f=StationaryScanDenoiser()
    for t in [1,1.1,1.2]:m=f.filter(scan(t,[4,.5,4]),True)
    assert list(m.ranges)==[4,.5,4]

def test_small_new_obstacle_limits_lag_to_two_frames():
    f=StationaryScanDenoiser()
    f.filter(scan(1,[1]),True);f.filter(scan(1.1,[1]),True)
    f.filter(scan(1.2,[.96]),True)
    assert abs(f.filter(scan(1.3,[.96]),True).ranges[0]-.96)<1e-5
