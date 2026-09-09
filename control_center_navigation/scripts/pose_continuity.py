"""Odometry-relative pose continuity guard; never alter TF or robot position."""
import math
import time
from collections import deque

def compose(a, b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2]

def relative(a,b):
    c,s=math.cos(a[2]),math.sin(a[2]);x,y=b[0]-a[0],b[1]-a[1]
    return c*x+s*y,-s*x+c*y,b[2]-a[2]

class PoseContinuityGuard:
    def __init__(self, distance=.5, angle=math.radians(15)):
        self.distance=distance;self.angle=angle;self.reset()

    def reset(self):
        self.history=deque();self.fault=''

    def update(self, stamp, map_pose, odom_pose):
        if self.fault:return False
        if not all(math.isfinite(v) for v in (stamp,*map_pose,*odom_pose)):
            self.fault='定位数据无效';return False
        if self.history and stamp < self.history[-1][0]:
            self.fault='定位时钟回退，需要重新定位';return False
        # Retain an anchor across a feedback gap; never silently accept the new
        # room/corridor as a baseline merely because a subscription was stale.
        while len(self.history)>1 and stamp-self.history[1][0]>1.0:
            self.history.popleft()
        for old_stamp, old_map, old_odom in self.history:
            predicted=compose(old_map,relative(old_odom,odom_pose))
            distance=math.hypot(map_pose[0]-predicted[0],map_pose[1]-predicted[1])
            angle=abs(math.atan2(math.sin(map_pose[2]-predicted[2]),math.cos(map_pose[2]-predicted[2])))
            dt=min(max(stamp-old_stamp,0.),1.)
            if distance>self.distance+.10*dt or angle>self.angle+.02*dt:
                self.fault=f'定位跳变 {distance:.2f}m / {math.degrees(angle):.1f}°；自动导航已阻止，请重新定位'
                return False
        self.history.append((stamp,map_pose,odom_pose))
        return True

class NavigationContinuityMonitor:
    def __init__(self,node):
        import json
        import tf2_ros
        from rclpy.time import Time
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import String
        self.node=node;self.json=json;self.Time=Time;self.String=String
        self.guard=PoseContinuityGuard();self.allowed=False;self.last_sample=0.;self.reset_until=0.
        self.status=None;self.last_status=0.
        self.buffer=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buffer,node)
        self.publisher=node.create_publisher(String,'/localization/continuity_status',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(PoseWithCovarianceStamped,'/initialpose',self._initial_pose,10)
        node.create_timer(.1,self._tick)

    def _initial_pose(self,msg):
        if msg.header.frame_id.strip('/')!='map':return
        self.guard.reset();self.allowed=False;self.reset_until=time.monotonic()+1.

    @staticmethod
    def pose(transform):
        p,q=transform.translation,transform.rotation
        return p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def _tick(self):
        now=time.monotonic();state='waiting';detail='等待新鲜定位和里程计'
        self.allowed=False
        try:
            if now<self.reset_until:raise ValueError('正在重新定位，暂缓自动导航')
            odom=self.buffer.lookup_transform('odom','base_link',self.Time())
            correction=self.buffer.lookup_transform('map','odom',self.Time())
            ros_now=self.node.get_clock().now().nanoseconds*1e-9
            for tf in (odom,correction):
                stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                if ros_now-stamp>1. or stamp-ros_now>2.:raise ValueError('定位/里程计已过期，暂缓自动导航')
            odom_pose=self.pose(odom.transform)
            self.allowed=self.guard.update(now,compose(self.pose(correction.transform),odom_pose),odom_pose)
            self.last_sample=now
            state='ok' if self.allowed else 'blocked'
            detail='定位连续性正常' if self.allowed else self.guard.fault
        except Exception as error:
            if self.guard.fault:state='blocked';detail=self.guard.fault
            elif isinstance(error,ValueError):detail=str(error)
        if (state,detail)!=self.status or now-self.last_status>1.:
            msg=self.String();msg.data=self.json.dumps(dict(state=state,detail=detail),ensure_ascii=False)
            self.publisher.publish(msg)
            if state=='blocked' and (not self.status or self.status[0]!='blocked'):
                self.node.get_logger().error(detail)
            self.status=(state,detail);self.last_status=now

    def permits_navigation(self):
        return self.allowed and time.monotonic()-self.last_sample<.5
