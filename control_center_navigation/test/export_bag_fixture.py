"""Read-only ROS bag conversion. No ROS publishers, no robot services."""
import math
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

rclpy.init()
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
buffer = Buffer(cache_time=Duration(seconds=60))
front, rear, grid = [], [], None
while reader.has_next():
    topic, data, _ = reader.read_next()
    msg = deserialize_message(data, types[topic])
    if topic in ('/tf', '/tf_static'):
        for transform in msg.transforms:
            if topic == '/tf_static': buffer.set_transform_static(transform, 'bag')
            else: buffer.set_transform(transform, 'bag')
    elif topic == '/map': grid = msg
    elif 'front_lidar' in topic: front.append(msg)
    elif 'rear_lidar' in topic: rear.append(msg)
assert grid is not None and front and rear
stamp = lambda scan: Time.from_msg(scan.header.stamp).nanoseconds
for index, fraction in enumerate((0.25, 0.50, 0.75)):
    f = front[int(len(front)*fraction)]
    r = min(rear, key=lambda scan: abs(stamp(scan)-stamp(f)))
    assert abs(stamp(r)-stamp(f)) <= 100_000_000
    points = []
    for scan in (f, r):
        t = buffer.lookup_transform_full('base_link', Time.from_msg(f.header.stamp),
            scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom')
        q = t.transform.rotation
        v = np.array([q.x, q.y, q.z])
        translation = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not max(.2, scan.range_min)<distance<min(15., scan.range_max): continue
            a=scan.angle_min+i*scan.angle_increment
            p=np.array([distance*math.cos(a),distance*math.sin(a),0.])
            p=p+2*q.w*np.cross(v,p)+2*np.cross(v,np.cross(v,p))+translation
            points.append((p[0],p[1]))
    origin=grid.info.origin
    q=origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    output=Path(sys.argv[2])/f'frame_{index}.txt'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(f'{grid.info.width} {grid.info.height} {grid.info.resolution} '
        f'{origin.position.x} {origin.position.y} {yaw}\n' +
        ' '.join(map(str,grid.data))+'\n'+str(len(points))+'\n'+
        '\n'.join(f'{x} {y}' for x,y in points)+'\n')
    print(f'{output}: points={len(points)}, capture_delta={abs(stamp(r)-stamp(f))/1e9:.6f}s')
rclpy.shutdown()
