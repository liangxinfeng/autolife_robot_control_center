"""Exercise UI methods without constructing ROS/Qt nodes."""
import ast,math
from pathlib import Path
from types import SimpleNamespace as NS

def test_display_uses_scan_timestamp_retries_and_expires():
    source=Path(__file__).resolve().parents[1]/'scripts/navigation_ui_node.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='NavigationRosNode')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_on_laser','_flush_laser_display','_laser_points_at_stamp')]
    tick=[10.];env={'math':math,'time':NS(monotonic=lambda:tick[0]),'Time':NS(from_msg=lambda x:x)}
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),env)
    outputs=[];calls=[];fail=[True]
    def lookup(target,frame,stamp):
        calls.append((target,frame,stamp.nanoseconds))
        if fail[0]:raise RuntimeError('TF not here yet')
        return NS(transform=NS(rotation=NS(x=0,y=0,z=0,w=1),translation=NS(x=2,y=3)))
    n=NS(_pending_laser={},_laser_by_topic={},_display_map_frame='map',tf_buffer=NS(lookup_transform=lookup),
         signals=NS(laser_received=NS(emit=outputs.append)),get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(tick[0]*1e9))))
    n._laser_points_at_stamp=lambda m,s:env['_laser_points_at_stamp'](n,m,s)
    m=NS(header=NS(stamp=NS(nanoseconds=int(9.9e9)),frame_id='laser'),ranges=[1.],range_min=.1,range_max=8.,angle_min=0.,angle_increment=.1)
    env['_on_laser'](n,'front',m);env['_flush_laser_display'](n)
    assert not outputs and len(n._pending_laser)==1
    fail[0]=False;tick[0]=10.1;env['_flush_laser_display'](n)
    assert calls[-1]==('map','laser',int(9.9e9)) and outputs[-1]==[(3.,3.)]
    tick[0]=10.7;env['_flush_laser_display'](n)
    assert outputs[-1]==[] and not n._laser_by_topic
    # Stale or missing stamps never fall back to latest TF.
    count=len(calls);m.header.stamp.nanoseconds=0
    env['_on_laser'](n,'front',m);env['_flush_laser_display'](n)
    assert len(calls)==count and not n._pending_laser
