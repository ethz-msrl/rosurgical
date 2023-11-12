import yaml

import rospy
from jsk_rviz_plugins.msg import OverlayText

from typing import Tuple, List
from std_msgs.msg import ByteMultiArray

def message_type_factory(msg_type: str) -> rospy.Message:
    valid_types = [ 'std_msgs/Float32', 
                    'std_msgs/String',
                    'geometry_msgs/Vector3', 
                    'mag_msgs/FieldStamped',
                    'mag_msgs/CurrentsStamped', 
                    'mag_msgs/FieldArrayStamped', 
                    'sensor_msgs/Image',
                    'sensor_msgs/CompressedImage',
                    'visualization_msgs/MarkerArray',
                    'geometry_msgs/TransformStamped',
                    'geometry_msgs/PoseStamped',
                    'byte_msgs/Floats32',
                    'sensor_msgs/Joy']

    assert msg_type in valid_types, f'message type {msg_type} must be one of the following: {valid_types}'

    if msg_type == 'std_msgs/Float32':
        from std_msgs.msg import Float32
        return Float32

    elif msg_type == 'std_msgs/String':
        from std_msgs.msg import String
        return String

    elif msg_type == 'Bytes':
        return Bytes
    
    elif msg_type == 'sensor_msgs/Joy':
        from sensor_msgs.msg import Joy
        return Joy

    elif msg_type == 'geometry_msgs/Vector3':
        from geometry_msgs.msg import Vector3
        return Vector3

    elif msg_type == 'mag_msgs/FieldStamped':
        from mag_msgs.msg import FieldStamped
        return FieldStamped

    elif msg_type == 'mag_msgs/FieldArrayStamped':
        from mag_msgs.msg import FieldArrayStamped
        return FieldArrayStamped
    
    elif msg_type == 'mag_msgs/CurrentsStamped':
        from mag_msgs.msg import CurrentsStamped
        return CurrentsStamped

    elif msg_type == 'nav_controller/currenterror':
        from nav_controller.msg import currenterror
        return currenterror

    elif msg_type == 'nav_controller/systemreport':
        from nav_controller.msg import systemreport
        return systemreport

    elif msg_type == 'sensor_msgs/Image':
        from sensor_msgs.msg import Image
        return Image

    elif msg_type == 'sensor_msgs/CompressedImage':
        from sensor_msgs.msg import CompressedImage
        return CompressedImage
    
    elif msg_type == 'nav_controller/Float32Stamped':
        from nav_controller.msg import Float32Stamped
        return Float32Stamped

    elif msg_type == 'visualization_msgs/MarkerArray':
        from visualization_msgs.msg import MarkerArray
        return MarkerArray

    elif msg_type == 'geometry_msgs/TransformStamped':
        from geometry_msgs.msg import TransformStamped
        return TransformStamped
    
    elif msg_type == 'geometry_msgs/PoseStamped':
        from geometry_msgs.msg import PoseStamped
        return PoseStamped
    
    elif msg_type == 'byte_msgs/Floats32':
        from byte_msgs.msg import Floats32
        return Floats32

def create_latency_overlay_msg(latency: float) -> OverlayText:
    """Creates an overlay text message for the communication latency.

    Args:
        latency (float): Communication latency in seconds

    Returns:
        OverlayText: Overlay text message
    """
    msg = OverlayText()
    msg.width = 200
    msg.height = 50
    msg.left = 0
    msg.top = 0
    msg.bg_color.r = 0.0
    msg.bg_color.g = 0.0
    msg.bg_color.b = 0.0
    msg.bg_color.a = 0.4
    msg.fg_color.r = 0.0
    msg.fg_color.g = 1.0
    msg.fg_color.b = 0.0
    msg.fg_color.a = 1.0
    msg.line_width = 2
    msg.text_size = 14
    msg.text = f"Latency: {latency*1000.0:3.1f} ms"
    return msg

def parse_topic_paramters(tcp_role: str, directory: str)-> Tuple[List]:
    """ Loads the specified yaml file and extracts the relevant information of the socket topics. 
    In particular, it extracts the topic names, types, lenghts as well as the ros role depending 
    on the specified tcp_role

    Args:
        tcp_role (string): 'host' or 'clients'
        directory (string): of the yaml file 

    Returns:
        topic_names (List): list of all topic names
        message_types (List): list of all message types
        message_lengths (List): list of message lengths
        ros_roles (List): List of all ros roles 
    """
    with open(directory, 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    topic_names = []
    message_types = []
    message_lengths = []
    ros_roles = []
    
    # Get all topic names
    for topic in data:
        # Get message type
        topic_names.append(topic)

        # Get message length
        msg_len = data[topic]["msg_len"]
        message_lengths.append(msg_len)

        # Get message length
        msg_type = data[topic]["msg_type"]
        message_types.append(msg_type)

        # Get ros role
        origin = data[topic]["origin"]
        assert origin in ["host", "client"]
        if origin == tcp_role:
            ros_role = "subscriber"
        else:
            ros_role = "publisher"
        ros_roles.append(ros_role)

    return (topic_names, message_types, message_lengths, ros_roles)

class Bytes(ByteMultiArray):
    def serialize(self, buff):
        """
        Function to convert ROS ByteMultiArray message to bytes buffer.
        Args:
        ros_msg (ByteMultiArray): ROS message

        Returns:
        bytes: bytes buffer
        """
        # Get data field from ROS message and convert to bytes
        buff = bytes(self.data)
        return buff
    
    def deserialize(self, str):
        # Convert string to bytes
        buffer = bytes.fromhex(buffer.replace('\\x', ''))

        # Create new ROS message
        ros_msg = ByteMultiArray()

        # Set data field of ROS message
        ros_msg.data = list(buffer)

        return ros_msg