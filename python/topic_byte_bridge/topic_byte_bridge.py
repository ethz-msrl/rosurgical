from io import BytesIO
import struct
import rospy 
from sockets.utils import parse_topic_paramters, message_type_factory
from std_msgs.msg import ByteMultiArray, Float32, String
from geometry_msgs.msg import PointStamped, PoseStamped
from mag_msgs.msg import FieldStamped
import tf
from byte_msgs.msg import Bytes, Floats32


class ClientBridge:
    def __init__(self, nh, tcp_role, yaml_dir: str) -> None:
        self.nh = nh
        self.tf_listener = tf.TransformListener()

        # Subsrciber
        self.incoming_bytes_sub = rospy.Subscriber(f'host_bytes', Floats32, self.byte_callback)
        self.mca_vel_sub = rospy.Subscriber('/navion/target_velocity_mca', Float32, self.mca_vel_callback)
        self.desired_field_sub = rospy.Subscriber('/backward_model/field', FieldStamped, self.desired_field_callback)

        # Publisher
        self.outgoing_byte_pub = rospy.Publisher('client_bytes', Floats32, queue_size=1)
        self.catheter_tip_proximal_pub = rospy.Publisher('/catheter_tip_proximal_', PointStamped, queue_size=1)
        self.catheter_tip_distal_pub = rospy.Publisher('/catheter_tip_distal_', PointStamped, queue_size=1)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1/500.0), self.timer_callback)

        # Messages 
        self.incoming_bytes_msg = Floats32()
        self.outgoing_bytes_msg = Floats32()
        self.mca_vel_msg = Float32()
        self.desired_field_msg = PointStamped()
        self.catheter_tip_proximal_msg = PointStamped()
        self.catheter_tip_distal_msg = PointStamped()

    def timer_callback(self, event):
        # Convert msgs to bytes
        #mca_vel_bytes = struct.pack('f', self.mca_vel_msg.data)
        #desired_field_bytes = struct.pack('fff', self.desired_field_msg.point.x, self.desired_field_msg.point.y, self.desired_field_msg.point.z)
        #total_bytes = mca_vel_bytes + desired_field_bytes

        # Print bytes
        #print(f"Total bytes: {total_bytes}")

        # Print bytes as integers
        #print([int(b) for b in total_bytes])
    

        #print(f'Raw bytes: {len(total_bytes)}')

        # Put bytes into outgoing message
        # self.outgoing_bytes_msg.data = total_bytes.decode('ISO-8859-1')
        self.outgoing_bytes_msg.data = [self.mca_vel_msg.data, self.desired_field_msg.point.x, self.desired_field_msg.point.y, self.desired_field_msg.point.z]
        # self.outgoing_bytes_msg.data = list(total_bytes)
        # print([b for b in self.outgoing_bytes_msg.data if not (0 <= b <= 255)])


        # Publish outgoing message
        self.outgoing_byte_pub.publish(self.outgoing_bytes_msg)

        # Serialize sent message and print its length
        #buff = BytesIO()
        #self.outgoing_bytes_msg.serialize(buff)
        #print(f'ROS serialize: {len(buff.getvalue())}')

    def byte_callback(self, msg: String):
        # Decode string to bytes
        self.incoming_bytes_msg = msg
        # incoming_bytes = self.incoming_bytes_msg.data.encode('ISO-8859-1')

        # Check if bytes have length of 24  
        # assert len(incoming_bytes) == 24, f'Incoming bytes have length {len(incoming_bytes)} instead of 24'

        # First three bytes are x,y,z of proximal tip
        proximal_float32 = msg.data[0:3]
        self.catheter_tip_proximal_msg.point.x, self.catheter_tip_proximal_msg.point.y, self.catheter_tip_proximal_msg.point.z = proximal_float32[0], proximal_float32[1], proximal_float32[2]
        self.catheter_tip_proximal_msg.header.stamp = rospy.Time.now()
        self.catheter_tip_proximal_msg.header.frame_id = 'mns'

        # Same for distal tip
        distal_tip_bytes = msg.data[3:6]
        self.catheter_tip_distal_msg.point.x, self.catheter_tip_distal_msg.point.y, self.catheter_tip_distal_msg.point.z = distal_tip_bytes[0], distal_tip_bytes[1], distal_tip_bytes[2]
        self.catheter_tip_distal_msg.header.stamp = rospy.Time.now()
        self.catheter_tip_distal_msg.header.frame_id = 'mns'

        # Publish
        self.catheter_tip_proximal_pub.publish(self.catheter_tip_proximal_msg)
        self.catheter_tip_distal_pub.publish(self.catheter_tip_distal_msg)
        
    def mca_vel_callback(self, msg):
        # Put float32 into outgoing message
        self.mca_vel_msg.data = msg.data

    def desired_field_callback(self, msg):
        # Trasnform field.vector to Point and transform to mns frame
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
        #outgoing_bytes = msg.data.encode('ISO-8859-1')
        # mca_vel, desired_field_x, desired_field_y, desired_field_z = struct.unpack('ffff', outgoing_bytes)
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
