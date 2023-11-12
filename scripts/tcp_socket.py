from abc import ABC, abstractmethod, abstractproperty
import socket
from typing import Callable
import rospy
from io import BytesIO

HEADER_SIZE = 30 # Length of header message sent
ENCODING = 'UTF-8'

"""
ToDo:
- Remove ACKs transmission for debugging
- Authentication and authorization
- Detect bandwidth (adjust video compression) dynamically
- Implement Service calls in a generic manner
- Encryption.
- Remove message dependencies. 
"""

def message_type_factory(msg_type: str) -> rospy.Message:
    valid_types = [ 'std_msgs/Float32', 
                    'std_msgs/String', 
                    'geometry_msgs/Vector3', 
                    'mag_msgs/FieldStamped',
                    'mag_msgs/CurrentsStamped', 
                    'mag_msgs/FieldArrayStamped', 
                    'nav_controller/currenterror', 
                    'nav_controller/systemreport',
                    'sensor_msgs/Image',
                    'sensor_msgs/CompressedImage',
                    'nav_controller/Float32Stamped']

    assert msg_type in valid_types, f'message type {message_type} must be one of the following: {valid_types}'

    if msg_type == 'std_msgs/Float32':
        from std_msgs.msg import Float32
        return Float32

    elif msg_type == 'std_msgs/String':
        from std_msgs.msg import String
        return String

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

class TCPSocket:
    @abstractmethod
    def __init__(self) -> None:
        pass

    @property
    @abstractmethod
    def communication_socket(self) -> socket.socket:
        pass

    def setup_ros(self, ros_role: str, topic_name: str, message_type: rospy.Message):
        """ Sets up ros objects for the socket. It determines whether it needs to subscribe or publish a topic and calls the proper callback.

        Args:
            ros_role (str): must be either publisher or subscriber
            topic_name (str): name of the topic to subscribe to or publish
            message_type (rospy.Message): type of message to be subscribed or published
        """
        
        self.msg_type = message_type

        # Case publisher
        if ros_role == 'publisher':
            self.pub = rospy.Publisher(topic_name, message_type, queue_size=1)
            self.publisher_loop()

        # Case subscriber
        elif ros_role == 'subscriber':
            self.sub = rospy.Subscriber(topic_name, message_type, self.subscriber_cb)
            rospy.spin()

    def publisher_loop(self):
        """Loop for publisher. Everytime the socket receives a message, the message will be deserialized and publsihed.

        """
        while not rospy.is_shutdown():
            # ToDo: ACKs can be removed eventually. Right now they are for debuggning purposes. 
            # Receive header
            header = self.communication_socket.recv(HEADER_SIZE)
            msg_len = int(header)

            # Ackknowledge header
            ack = True
            ack = bytes(ack)
            self.communication_socket.send(ack)

            # Reveice message
            msg_bytes = None
            i = 0
            while True:
                data = self.communication_socket.recv(msg_len)
                if not msg_bytes:
                    msg_bytes = data
                else:
                    msg_bytes += bytearray(data)
                i += 1
                if not data or len(msg_bytes) == msg_len: 
                    break
            #rospy.loginfo(f"{self.topic_name} with {len(msg_bytes)} was received in {i} transmissions")

            # Acknowledge message 
            ack = True
            ack = bytes(ack)
            self.communication_socket.send(ack)

            # Decode message
            msg = self.msg_type().deserialize(msg_bytes)

            # Publioh message
            self.pub.publish(msg)


    def subscriber_cb(self, msg: rospy.Message):
        """Subscriber callback in case the socket acts as a subscriber. Everytime the subsriber receives a message, it is serialized and sent via the socket to the receipient. 

        Args:
            msg (rospy.Message): message received by the subscriber. 
        """
        # Send header to server
        buffer = BytesIO()
        msg.serialize(buffer)
        buffer = buffer.getvalue()
        msg_len = len(buffer)
        header = f'{msg_len:<{HEADER_SIZE}}'
        self.communication_socket.send(header.encode(ENCODING))

        # Wait for server to ackknowledge the header
        ack = self.communication_socket.recv(4)
        assert bool(ack.decode(ENCODING)) == True

        # Put message into buffer and send it to the server
        self.communication_socket.send(buffer)

        # Wait for server to ackknowledge the header
        ack = self.communication_socket.recv(4)
        print(ack.decode(ENCODING))
        assert bool(ack.decode(ENCODING)) == True
        
class TCPServer(TCPSocket):
    def __init__(self, host: str, port:str, message_type: rospy.Message, ros_role: str, topic_name) -> None:
        """Constructor of a TCPServer for ros communication. 

        Args:
            host (str): IP address of the host computer or the gate if port forwarding is used
            port (str): communication port
            message_type (rospy.Message): type of message to be transmitted
            ros_role (str): subscriber or publisher
            topic_name (_type_): topic name to publish or subscribe to
        """
        # Initialize host
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        rospy.loginfo(f'Server {host} on port {port} launched.')

        # Listen for client
        self.socket.listen(1)
        rospy.loginfo(f'Server {host} listening to port {port}.')

        # Accept client connection
        # ToDo: Have some kind of authentication?
        self.client_socket, self.client_address = self.socket.accept()
        rospy.loginfo(f'Connection from {self.client_address} has been established.')

        # Setup ros
        self.topic_name = topic_name
        self.setup_ros(ros_role, topic_name, message_type)
        
    @property
    def communication_socket(self) -> socket.socket:
        """
        Returns:
            socket.socket: socket object to be used for communication. 
        """
        return self.client_socket

class TCPClient(TCPSocket):
    def __init__(self, host: str, port:str, message_type: rospy.Message, ros_role: str, topic_name: str) -> None:
        """Constructor of a TCPClient for ros communication. 

        Args:
            host (str): IP address of the host computer or the gate if port forwarding is used
            port (str): communication port
            message_type (rospy.Message): type of message to be transmitted
            ros_role (str): 'subscriber' or 'publisher'
            topic_name (str): If socket acts as subscriber it is the name of the topic to subscribe to, otherwise it is the name of the topic to be published
        """
        # Initialize socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket.setblocking(1) 

        # Connect to host
        self.socket.connect((host, port))
        rospy.loginfo(f'Successfully connected {host} on port {port}.')

        # Setup ros
        self.topic_name = topic_name
        self.setup_ros(ros_role, topic_name, message_type)

    @property
    def communication_socket(self) -> socket.socket:
        """
        Returns:
            socket.socket: socket object to be used for communication. 
        """
        return self.socket


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('tcp_socket')

    # Get all relevant parameters 
    host = rospy.get_param('~host', None)
    port = rospy.get_param('~port', None)
    assert port >= 3000 and port <= 3005, 'Port should be between 3000 and 3005. These ports are opened for the Navion.'
    topic_name = rospy.get_param('~topic_name', None)
    message_type = rospy.get_param('~message_type', None)
    publish_latency = rospy.get_param('~publish_latency', False)
    is_dynamic_length = rospy.get_param('~is_dynamic_length', False) # not used right now - TBI. Can be used to avoid header transmission for messages of static length
    tcp_role = rospy.get_param('~tcp_role', None) 
    assert tcp_role in ['host', 'client']
    ros_role = rospy.get_param('~ros_role', None) 
    assert ros_role in ['publisher', 'subscriber'], f'Invalid ros role given. Must be either in {ros_role}'

    # Get message_type type
    message_type = message_type_factory(message_type)
    try:
        if tcp_role == 'host':
            socket = TCPServer(host, port, message_type, ros_role, topic_name)
        elif tcp_role == 'client':
            socket = TCPClient(host, port, message_type, ros_role, topic_name)

    except:
        socket.socket.close(), f"Closing socket {host, port}"
    
    socket.socket.close(), f"Closing socket {host, port}"
