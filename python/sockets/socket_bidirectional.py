import os
import ssl
import time
import socket
from io import BytesIO
from typing import Callable, Dict, List
from functools import partial
from abc import ABC, abstractmethod, abstractproperty

import rospy
from std_msgs.msg import Float32
from jsk_rviz_plugins.msg import OverlayText
from sockets.utils import message_type_factory, create_latency_overlay_msg


class BidirectionalSocket:
    def __init__(self, host: str, port:str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str], msg_lens: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        """
        Constructor of a bidirectional socket for ros communication. This socket can transfer several topics at the same time.
        
        Args:
            host (str): IP address of the host computer or the gate if port forwarding is used
            port (str): communication port
            message_types (List[rospy.Message]): List of all ros message types to be transferred
            ros_role (List[str]): List of all ros roles. A role can either be 'subscriber' or 'publisher'
            topic_names (List[str]): List of all topic names. If socket acts as subscriber it is the name of the topic to subscribe to, otherwise it is the name of the topic to be published.
            msg_lens (List[str]): List of all topic lengths. We are using a fixed length message format to avoid the overhead of sending the length of the message. 
        """

        self.all_sent = False
        self.all_reveived = False

        # General attributes
        self.host = host
        assert len(message_types) == len(topic_names)
        self.topic_names = topic_names
        self.message_types = {topic: message_type_factory(msg_type) for topic, msg_type in zip(topic_names, message_types)}
        self.message_lens = {topic: int(msg_len) for topic, msg_len in zip(topic_names, msg_lens)}
        self.msg_len = 0
        self.recevie_list = [0]
        self.ros_roles = ros_roles
        
        # Ros attributes
        self.subscribers = {}
        self.subscriber_msgs = {}
        self.subscriber_lens = {}
        self.publishers = {}
        self.publishers_lens = {}
        
        # Latency attributes
        self.com_lat_pub = rospy.Publisher("communication_latency", OverlayText, queue_size=1)
        self.com_lat_float_pub = rospy.Publisher("communication_latency_float", Float32, queue_size=1)
        self.com_lat = 0.0
        
        self.setup_ros()

    def setup_ros(self):
        """Sets up ros subscribers and publishers based on the given ros roles and topic names.
        """
        for topic, ros_role in zip(self.topic_names, self.ros_roles): 
            message_type = self.message_types[topic]
            if ros_role == 'subscriber':
                subscriber = rospy.Subscriber(topic, message_type, partial(self.cb, topic), queue_size=1)
                self.subscribers[topic] = subscriber
                self.subscriber_msgs[topic] = None
            elif ros_role == 'publisher':
                publisher = rospy.Publisher(f'{topic}', message_type, queue_size=1)
                self.publishers[topic] = publisher
                self.msg_len += self.message_lens[topic]
                self.recevie_list.append(self.recevie_list[-1]+self.message_lens[topic])
    
    def cb(self, topic_name: str, msg: rospy.Message):
        """Subscriber callback. Can be used as callback for a specific subscriber by defining a partial function w/ the respective topic_name.

        Args:
            topic_name (str): Name of the topic
            msg (rospy.Message): Message to be processed
        """
        # Save message
        self.subscriber_msgs[topic_name] = msg

        # Only send and receive messages if all messages have been received
        if self.check_all_msgs_received() and self.all_sent == True and self.all_reveived == True:
            # Block other callbacks
            self.all_sent = False
            self.all_reveived = False

            # Start timer 
            start = time.time()

            # Send all messages
            self.send_msgs()
            self.all_sent = True

            # Receive all messages
            self.receive_messages()
            self.all_reveived = True

            # Time roundtrip
            end = time.time()
            self.com_lat = end - start

            # Publish communication latency
            self.publish_com_lat()
            
    def publish_com_lat(self)-> None:
        """Publishes the communication latency as an overlay text and as a float.
        """
        msg = create_latency_overlay_msg(self.com_lat)
        self.com_lat_pub.publish(msg)
        self.com_lat_float_pub.publish(Float32(self.com_lat*1000.0))

    def check_all_msgs_received(self)-> bool:
        """Checks if all registered messages have been received.
        """
        for key in self.subscriber_msgs:
            if self.subscriber_msgs[key] == None:
                return False
        return True

    def zero_messages(self, dictionary: Dict)-> None:
        """Sets all values of a dictionary to None. This is done to reset the messages after they have been sent.

        Args:
            dictionary (Dict): diytionary to be reset
        """
        for key in dictionary:
            dictionary[key] = None

    def send_msgs(self)-> None:
        """Sends all messages to the other side of the socket.
        """

        # Initialize buffer
        total_buffer = None

        # Assemble buffer
        for topic_name in self.subscriber_msgs:
            byte_array = bytearray(self.message_lens[topic_name])
            temp_buffer = BytesIO()
            msg = self.subscriber_msgs[topic_name]
            msg.serialize(temp_buffer)
            temp_buffer = temp_buffer.getvalue()
            
            # Check that the message length is not exceeding the maximum length
            msg_len = len(temp_buffer)
            assert msg_len <= self.message_lens[topic_name], f'{topic_name} was set to have length of {self.message_lens[topic_name]} but has {msg_len} instead. Please correct it in your launch file'

            # Add message to buffer
            byte_array[:msg_len] = temp_buffer
            if total_buffer is None:
                total_buffer = byte_array
            else:
                total_buffer += byte_array
  
        # Send buffer and reset dictionary
        self.communication_socket.send(total_buffer)
        # self.zero_messages(self.subscriber_msgs)

        
    def receive_messages(self):
        """Receives all messages from the other side of the socket.
        """
        # Initialize buffer
        msg_bytes = None
        i = 0

        # Receive buffer
        while True:
            data = self.communication_socket.recv(self.msg_len)
            if not msg_bytes:
                msg_bytes = data
            else:
                msg_bytes += bytearray(data)
            i += 1
            if not data or len(msg_bytes) == self.msg_len: 
                break
        
        # Decode buffer
        for i, topic_name in enumerate(self.publishers):
            start = self.recevie_list[i]
            end = self.recevie_list[i+1]
            temp_byte = msg_bytes[start:end]
            
            # Decode message
            msg = self.message_types[topic_name]().deserialize(temp_byte)

            # Publioh message
            self.publishers[topic_name].publish(msg)

    def wrap_socket_ssl_context(self, socket: socket.socket, cert_path: str, key_path: str, cert_verify_path: str, is_server: bool):
        # Check that certificates and key files exist.
        assert os.path.exists(cert_path), f"Certificate could not be found {cert_path}"
        assert os.path.exists(key_path), f"Key could not be found {key_path}"
        assert os.path.exists(cert_verify_path), f"Verification certificate could not be found {cert_verify_path}"

        # Create ssl context 
        protocol = ssl.PROTOCOL_TLS_SERVER if is_server else ssl.PROTOCOL_TLS_CLIENT
        context = ssl.SSLContext(protocol)
        context.verify_mode = ssl.CERT_REQUIRED
        context.load_cert_chain(cert_path, key_path)
        context.load_verify_locations(cert_verify_path)

        # Wrap socket
        try:
            if is_server:
                ssl_socket = context.wrap_socket(socket, server_side=True)
            else:
                context.check_hostname = False
                ssl_socket = context.wrap_socket(socket, server_hostname=self.host) 
        except Exception as e:
            raise ConnectionError(e)
        
        return ssl_socket

class BidirectionalClient(BidirectionalSocket):
    def __init__(self, host: str, port: str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str] , msg_lens: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        super().__init__(host, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)

        # Initialize client
        self.socket_no_ssl = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket_no_ssl.setblocking(True)

        # Check if ssl requested
        paths = [cert_path, key_path, cert_verify_path]
        wrap_ssl = all(path is not None for path in paths)

        if wrap_ssl:
            rospy.loginfo("Establishing SSL/TLS connection")
            self.socket = self.wrap_socket_ssl_context(self.socket_no_ssl, cert_path, key_path, cert_verify_path, False)
        else:
            rospy.loginfo("Establishing TCP/IP connection without SSL")
            self.socket = self.socket_no_ssl
        # Connect to host
        self.socket.connect((host, port))
        rospy.sleep(1)
        rospy.loginfo(f'Successfully connected {host} on port {port}.')

        # Set flags        
        self.all_sent = True
        self.all_reveived = True

        # Start listening
        rospy.spin()

    @property
    def communication_socket(self) -> socket.socket:
        """
        Returns:
            socket.socket: socket object to be used for communication. 
        """
        return self.socket

class BidirectionalHost(BidirectionalSocket):
    def __init__(self, host: str, port: str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str], msg_lens: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        super().__init__(host, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)

        # Initialize host
        self.socket_no_ssl = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket_no_ssl.setblocking(True)
        self.socket_no_ssl.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_no_ssl.bind((host, port))
        rospy.loginfo(f'Server {host} on port {port} launched.')

        # Check if ssl requested
        paths = [cert_path, key_path, cert_verify_path]
        wrap_ssl = all(path is not None for path in paths)

        # Wrap socket if requested
        if wrap_ssl:
            rospy.loginfo("Establishing SSL/TLS connection")
            self.socket = self.wrap_socket_ssl_context(self.socket_no_ssl, cert_path, key_path, cert_verify_path, True)
        else:
            rospy.loginfo("Establishing TCP/IP connection without SSL.")
            self.socket = self.socket_no_ssl

        # Listen for client
        self.socket.listen(1) # Allow single connection only
        rospy.loginfo(f'Server {host} listening to port {port}.')

        # Accept client connection
        self.client_socket, self.client_address = self.socket.accept()
        rospy.sleep(2)
        rospy.loginfo(f'Connection from {self.client_address} has been established.')

        # Set flags        
        self.all_sent = True
        self.all_reveived = True

        # Receive messages from client
        self.receive_messages()

        # Start listening
        rospy.spin()

    @property
    def communication_socket(self) -> socket.socket:
        """
        Returns:
            socket.socket: socket object to be used for communication. 
        """
        return self.client_socket