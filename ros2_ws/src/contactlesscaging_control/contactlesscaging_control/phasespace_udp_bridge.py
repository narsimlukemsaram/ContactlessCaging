"""Bridge the binary PhaseSpace UDP tracker packet from the historical C++ source.

Wire format used by the repository's PhaseSpace C++ example:
  int32 count; repeated {char name[32]; struct TrackerState}
  TrackerState = float position[3], float pose[4], float headingY,
                 4 * {uint32 marker_id, float x, y, z, timestamp}.

Configure coordinate scaling and heading units for the actual tracking stream.
This parser intentionally checks packet bounds and ignores malformed data.
"""
from __future__ import annotations
import math
import socket
import struct
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .legacy_packet import decode_packet, TRACKER_SIZE


class PhaseSpaceUDPBridge(Node):
    def __init__(self):
        super().__init__('phasespace_udp_bridge')
        self.declare_parameter('bind_host','0.0.0.0')
        self.declare_parameter('udp_port',7777)
        self.declare_parameter('position_scale',1.0)
        self.declare_parameter('heading_unit','radians')
        self.declare_parameter('frame_id','world')
        self.declare_parameter('tracker_names',['AcoustoBot1','AcoustoBot2','AcoustoBot3','AcoustoBot4'])
        self.declare_parameter('robot_names',['robot1','robot2','robot3','robot4'])
        p=lambda k:self.get_parameter(k).value
        names=list(p('tracker_names')); robots=list(p('robot_names'))
        if len(names)!=len(robots): raise ValueError('tracker_names and robot_names must match')
        self.mapping=dict(zip(names,robots))
        self.scale=float(p('position_scale'))
        self.heading_unit=str(p('heading_unit')).lower()
        if self.heading_unit not in ('radians','degrees'):
            raise ValueError('heading_unit must be radians or degrees')
        self.frame=str(p('frame_id'))
        self.pubs={robot:self.create_publisher(PoseStamped,f'/phasespace/{robot}/pose',10) for robot in robots}
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind((str(p('bind_host')),int(p('udp_port'))))
        self.sock.setblocking(False)
        self.create_timer(0.01,self._poll)
        self.get_logger().info(f'Listening to legacy PhaseSpace UDP packet on port {p("udp_port")}')

    def _poll(self):
        for _ in range(20):
            try:
                data,_=self.sock.recvfrom(8192)
            except BlockingIOError:
                return
            except OSError as e:
                self.get_logger().error(str(e));return
            try:
                records=decode_packet(data)
            except ValueError:
                continue
            for name,x,y,z,heading in records:
                robot=self.mapping.get(name)
                if robot is None:continue
                yaw=math.radians(heading) if self.heading_unit=='degrees' else heading
                msg=PoseStamped()
                msg.header.stamp=self.get_clock().now().to_msg()
                msg.header.frame_id=self.frame
                msg.pose.position.x=x*self.scale
                msg.pose.position.y=y*self.scale
                msg.pose.position.z=z*self.scale
                msg.pose.orientation.z=math.sin(yaw/2.0)
                msg.pose.orientation.w=math.cos(yaw/2.0)
                self.pubs[robot].publish(msg)

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=PhaseSpaceUDPBridge()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
