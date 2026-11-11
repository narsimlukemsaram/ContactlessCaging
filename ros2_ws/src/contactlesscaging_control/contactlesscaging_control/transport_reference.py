"""Publish a straight 1 m commanded leader reference at nominal 5 cm/s.
The start is latched from the first fresh leader pose, after an explicit start command.
Call: ros2 service call /formation/start std_srvs/srv/Trigger '{}'
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class TransportReference(Node):
    def __init__(self):
        super().__init__('transport_reference')
        self.declare_parameter('leader_name','robot1')
        self.declare_parameter('path_length_m',1.0)
        self.declare_parameter('commanded_speed_mps',0.05)
        self.declare_parameter('direction_x',1.0)
        self.declare_parameter('direction_y',0.0)
        self.declare_parameter('frame_id','world')
        p=lambda k:self.get_parameter(k).value
        self.p=p; self.pose=None;self.pose_time=None;self.initial=None;self.started=None
        self.pub=self.create_publisher(PoseStamped,'/formation/leader_target',10)
        self.create_subscription(PoseStamped, f'/phasespace/{p("leader_name")}/pose', self._pose,10)
        self.create_service(Trigger, '/formation/start', self._start)
        self.create_timer(0.05,self._tick)
    def _pose(self,msg):
        if msg.header.frame_id and msg.header.frame_id != str(self.p('frame_id')): return
        self.pose=msg;self.pose_time=self.get_clock().now().nanoseconds * 1e-9
    def _start(self,req,res):
        del req
        now=self.get_clock().now().nanoseconds * 1e-9
        if self.pose is None or self.pose_time is None or now-self.pose_time > 0.5:
            res.success=False;res.message='A fresh world-frame leader pose is required';return res
        if self.started is not None:
            res.success=False;res.message='Reference is already running';return res
        self.initial=(self.pose.pose.position.x,self.pose.pose.position.y)
        self.started=now
        res.success=True;res.message='One-metre reference activated';return res
    def _tick(self):
        if self.started is None:return
        import math
        now=self.get_clock().now().nanoseconds*1e-9
        speed=max(0.0,float(self.p('commanded_speed_mps')))
        length=max(0.0,float(self.p('path_length_m')))
        s=min(length, speed*max(0.0,now-self.started))
        dx=float(self.p('direction_x'));dy=float(self.p('direction_y'))
        norm=math.hypot(dx,dy)
        if norm <= 1e-9:return
        msg=PoseStamped();msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id=str(self.p('frame_id'))
        msg.pose.position.x=self.initial[0]+s*dx/norm
        msg.pose.position.y=self.initial[1]+s*dy/norm
        msg.pose.orientation.w=1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args);n=TransportReference()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
