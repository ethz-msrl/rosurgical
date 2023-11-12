#!/usr/bin/env python3

import rospy
from io import BytesIO
import struct
import rospy 
from sockets.utils import parse_topic_paramters, message_type_factory
from std_msgs.msg import ByteMultiArray, Float32, String
from geometry_msgs.msg import PointStamped, PoseStamped
from mag_msgs.msg import FieldStamped
import tf
from byte_msgs.msg import Floats32


class ClientBridge:
    def __init__(self, nh, tcp_role, yaml_dir: str) -> None:
        self.nh = nh
        self.tf_listener = tf.TransformListener()

        # Messages 
        self.incoming_bytes_msg = Floats32()
        self.outgoing_bytes_msg = Floats32()
        self.mca_vel_msg = Float32()
        self.desired_field_msg = PointStamped()
        self.tip_pose_msg = PoseStamped()

        # Publisher
        self.outgoing_byte_pub = rospy.Publisher('/client_bytes', Floats32, queue_size=1)
        self.tip_psoe_pub = rospy.Publisher('/tip_pose', PoseStamped, queue_size=1)

        # Subsrciber
        self.incoming_bytes_sub = rospy.Subscriber(f'host_bytes', Floats32, self.byte_callback)
        self.mca_vel_sub = rospy.Subscriber('/navion/target_velocity_mca', Float32, self.mca_vel_callback)
        self.desired_field_sub = rospy.Subscriber('/backward_model/field', FieldStamped, self.desired_field_callback)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1/500.0), self.timer_callback)

    def timer_callback(self, event):
        self.outgoing_bytes_msg.data = [self.mca_vel_msg.data, self.desired_field_msg.point.x, self.desired_field_msg.point.y, self.desired_field_msg.point.z]
        self.outgoing_byte_pub.publish(self.outgoing_bytes_msg)
        self.tip_psoe_pub.publish(self.tip_pose_msg)

    def byte_callback(self, msg: String):
        self.incoming_bytes_msg = msg
        self.tip_pose_msg.pose.position.x, self.tip_pose_msg.pose.position.y, self.tip_pose_msg.pose.position.z = msg.data[0], msg.data[1], msg.data[2]
        self.tip_pose_msg.pose.orientation.x, self.tip_pose_msg.pose.orientation.y, self.tip_pose_msg.pose.orientation.z, self.tip_pose_msg.pose.orientation.w = msg.data[3], msg.data[4], msg.data[5], msg.data[6]
        self.tip_pose_msg.header.stamp = rospy.Time.now()
        self.tip_pose_msg.header.frame_id = 'mns'
        self.tip_psoe_pub.publish(self.tip_pose_msg)
        
    def mca_vel_callback(self, msg):
        self.mca_vel_msg.data = msg.data

    def desired_field_callback(self, msg):
        field_point = PointStamped()
        field_point.header = msg.header
        field_point.header.stamp = rospy.Time(0)
        field_point.point = msg.field.vector
        field_point = self.tf_listener.transformPoint('mns', field_point)
        self.desired_field_msg = field_point

class HostBridge:
    def __init__(self, nh, tcp_role, yaml_dir: str) -> None:
        self.nh = nh
        self.tf_listener = tf.TransformListener()

        # Subscriber
        self.outgoing_bytes_sub = rospy.Subscriber('client_bytes', Floats32, self.byte_callback)
        self.catheter_tip_proximal_sub = rospy.Subscriber('/tip_pose', PoseStamped, self.tip_pose_callback)

        # Publisher
        self.incoming_byte_pub = rospy.Publisher('host_bytes', Floats32, queue_size=1)
        self.mca_vel_pub = rospy.Publisher('/navion/target_velocity_mca', Float32, queue_size=1)
        self.desired_field_pub = rospy.Publisher('/backward_model/field', FieldStamped, queue_size=1)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1/500.0), self.timer_callback)

        # Messages
        self.mca_vel_msg = Float32()
        self.desired_field_msg = FieldStamped()
        self.incoming_bytes_msg = Floats32()
        self.tip_pose_data = [0.0 for i in range(7)]

    def timer_callback(self, event):
        self.incoming_bytes_msg.data = self.tip_pose_data
        self.incoming_byte_pub.publish(self.incoming_bytes_msg)

    def byte_callback(self, msg: Floats32):
        self.mca_vel_msg.data = msg.data[0]
        self.desired_field_msg.field.vector.x = msg.data[1]
        self.desired_field_msg.field.vector.y = msg.data[2]
        self.desired_field_msg.field.vector.z = msg.data[3]
        self.desired_field_msg.header.stamp = rospy.Time.now()
        self.desired_field_msg.header.frame_id = 'mns'
        self.mca_vel_pub.publish(self.mca_vel_msg)
        self.desired_field_pub.publish(self.desired_field_msg)

    def tip_pose_callback(self, msg: PoseStamped):
        self.tip_pose_data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


if __name__ == '__main__':
    nh = rospy.init_node('topic_byte_bridge_node')
    tcp_role = rospy.get_param('~tcp_role', None)

    if tcp_role == 'host':
        host_bridge = HostBridge(nh, tcp_role='host', yaml_dir='../../config/topic_byte_bridge.yaml')
    elif tcp_role == 'client':
        host_bridge = ClientBridge(nh, tcp_role='client', yaml_dir='../../config/topic_byte_bridge.yaml')
    else:
        raise ValueError(f'Invalid tcp_role: {tcp_role}')
    rospy.spin()