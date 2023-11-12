import struct
from std_msgs.msg import ByteMultiArray, Float32
from mag_msgs.msg import FieldStamped

def float32_to_bytes(float32):
    """
    Function to convert float32 to bytes.
    Args:
    float32 (float32): float32 value

    Returns:
    bytes: bytes buffer
    """
    # Convert float32 to bytes
    buffer = struct.pack('f', float32)

    return buffer

def bytes_to_float32(buffer):
    """
    Function to convert bytes to float32.
    Args:
    buffer (bytes): bytes buffer

    Returns:
    float32: float32 value
    """
    # Convert bytes to float32
    float32 = struct.unpack('f', buffer)[0]

    return float32  

def field_to_bytes(field):
    """
    Function to convert ROS FieldStamped message to bytes buffer.
    Args:
    field (FieldStamped): ROS message

    Returns:
    bytes: bytes buffer
    """
    # Put x y z of the field in a buffer of 12 bytes
    buffer = struct.pack('fff', field.x, field.y, field.z)

    return buffer

def bytes_to_field(buffer):
    """
    Function to convert bytes to ROS FieldStamped message.
    Args:
    buffer (bytes): bytes buffer

    Returns:
    FieldStamped: ROS message
    """
    # Convert bytes to float32
    x, y, z = struct.unpack('fff', buffer)

    # Create new ROS message
    field = FieldStamped()

    # Set data field of ROS message
    field.x = x
    field.y = y
    field.z = z

    return field

def byte_multi_array_to_bytes(ros_msg):
    """
    Function to convert ROS ByteMultiArray message to bytes buffer.
    Args:
    ros_msg (ByteMultiArray): ROS message

    Returns:
    bytes: bytes buffer
    """
    # Get data field from ROS message and convert to bytes
    buffer = bytes(ros_msg.data)

    return buffer

def bytes_to_byte_multi_array(buffer):
    """
    Function to convert bytes to ROS ByteMultiArray message.
    Args:
    buffer (bytes): bytes buffer

    Returns:
    ByteMultiArray: ROS message
    """
    # Convert string to bytes
    buffer = bytes.fromhex(buffer.replace('\\x', ''))

    # Create new ROS message
    ros_msg = ByteMultiArray()

    # Set data field of ROS message
    ros_msg.data = list(buffer)

    return ros_msg