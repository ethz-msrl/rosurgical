"""
BSD 3-Clause License

Copyright (c) 2023, ETHZ MSRL

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import ssl
import time
import struct
import socket
from io import BytesIO
from typing import Dict, List
from functools import partial

import rospy
from std_msgs.msg import Float32
from jsk_rviz_plugins.msg import OverlayText
from rosurgical_lib.utils import message_type_factory, create_latency_overlay_msg


class ROSurgicalSocket:
    def __init__(self, hostname: str, port:str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        """
        Constructor of a ROSurgical socket for ROS communication. This socket can transfer several topics at the same time.
        
        Args:
            hostname (str): IP address of the host computer or the gate if port forwarding is used.
            port (str): Communication port.
            message_types (List[rospy.Message]): List of all ROS message types to be transferred.
            topic_names (List[str]): List of all topic names. If socket acts as a subscriber, it is the name of the topic to subscribe to, otherwise it is the name of the topic to be published.
            ros_roles (List[str]): List of all ROS roles. A role can either be 'subscriber' or 'publisher'.
            cert_path (str): Path to SSL certificate.
            key_path (str): Path to private key.
            cert_verify_path (str): Path to verification certificate of the opposite host.
        """

        # Initialize general attributes for the socket
        self.all_sent = False
        self.all_received = False
        self.hostname = hostname
        # Ensure that the number of message types matches the number of topic names
        assert len(message_types) == len(topic_names)
        self.topic_names = topic_names
        # Create a dictionary mapping topic names to their corresponding message types
        self.message_types = {topic: message_type_factory(msg_type) for topic, msg_type in zip(topic_names, message_types)}
        # Initialize a dictionary to keep track of message lengths for each topic
        self.message_lens = {topic: None for topic in topic_names}
        self.msg_len = 0
        self.receive_list = [0]
        self.ros_roles = ros_roles
        
        # Initialize ROS communication attributes
        self.subscribers = {}
        self.subscriber_msgs = {}
        self.subscriber_lens = {}
        self.publishers = {}
        self.publishers_lens = {}
        
        # Initialize latency measurement attributes
        self.com_lat_pub = rospy.Publisher("communication_latency", OverlayText, queue_size=1)
        self.com_lat_float_pub = rospy.Publisher("communication_latency_float", Float32, queue_size=1)
        self.com_lat = 0.0
        
        # Setup ROS components (subscriptions, publications, etc.)
        self.setup_ros()

    def setup_ros(self):
        """
        Sets up ROS subscribers and publishers based on the given ROS roles and topic names.
        This method iterates over each topic and its corresponding role, setting up either a 
        subscriber or publisher as appropriate.
        """
        for topic, ros_role in zip(self.topic_names, self.ros_roles): 
            # Retrieve the message type for the current topic
            message_type = self.message_types[topic]

            if ros_role == 'subscriber':
                # Set up a ROS subscriber for the topic
                subscriber = rospy.Subscriber(topic, message_type, partial(self.cb, topic), queue_size=1)
                self.subscribers[topic] = subscriber
                # Initialize a placeholder for incoming messages
                self.subscriber_msgs[topic] = None
            elif ros_role == 'publisher':
                # Set up a ROS publisher for the topic
                publisher = rospy.Publisher(f'{topic}', message_type, queue_size=1)
                self.publishers[topic] = publisher
        
        # Initialize message lengths for each topic
        self.get_local_msg_lengths()
    
    def cb(self, topic_name: str, msg: rospy.Message):
        """
        Subscriber callback. Can be used as a callback for a specific subscriber by defining 
        a partial function with the respective topic_name.

        This callback is triggered whenever a message is received on a subscribed topic. 
        It processes the message and manages the sending and receiving of all messages, 
        ensuring synchronization and measuring communication latency.

        Args:
            topic_name (str): Name of the topic on which the message is received.
            msg (rospy.Message): Message received from the topic.
        """
        # Store the received message in the corresponding topic's message buffer
        self.subscriber_msgs[topic_name] = msg

        # Check if all messages have been received and sent
        if self.check_all_msgs_received() and self.all_sent == True and self.all_received == True:
            # Reset flags to block other callbacks
            self.all_sent = False
            self.all_received = False

            # Start timing the round trip for latency measurement
            start = time.time()

            # Send all pending messages
            self.send_msgs()
            self.all_sent = True

            # Receive all incoming messages
            self.receive_messages()
            self.all_received = True

            # Calculate the total round-trip time
            end = time.time()
            self.com_lat = end - start

            # Publish the calculated communication latency
            self.publish_com_lat()
            
    def publish_com_lat(self)-> None:
        """
        Publishes the communication latency as an overlay text and as a float.
        This method is used to display the latency in the ROS environment, both visually 
        in RViz and as a numeric value for logging or further processing.
        """
        # Create an OverlayText message with the current communication latency
        msg = create_latency_overlay_msg(self.com_lat)
        # Publish the latency as an overlay text for RViz visualization
        self.com_lat_pub.publish(msg)
        
        # Publish the latency as a Float32 message for numerical use
        self.com_lat_float_pub.publish(Float32(self.com_lat*1000.0))

    def check_all_msgs_received(self)-> bool:
        """
        Checks if all registered messages have been received.
        This method iterates through all the messages in the subscribers' message 
        buffers to determine if each has received a message since the last reset.

        Returns:
            bool: True if all messages have been received, False otherwise.
        """
        # Iterate through all subscriber message buffers
        for key in self.subscriber_msgs:
            # Check if any buffer is empty (None)
            if self.subscriber_msgs[key] == None:
                return False # Not all messages have been received
            
        return True # All messages have been received
    
    def get_local_msg_lengths(self)-> bool:
        """
        Determines the length of each message type that the socket subscribes to.
        This method is used to calculate and store the expected length of each message
        to facilitate efficient data transfer.
        """
        for key in self.subscribers:
            # Wait for a message of each type to calculate its length
            msg = rospy.wait_for_message(key, self.message_types[key])
            # Convert the ROS message to bytes
            temp_buffer = self.ros_msg_to_bytes(msg)
            # Calculate the message length with a buffer margin
            msg_len = len(temp_buffer)
            self.message_lens[key] = int(1.2*(msg_len+1)) # Adding a buffer margin

    def receive_msg_lengths(self)-> bool:
        """
        Receives the lengths of messages that the socket will publish.
        This method is used to receive the length of each message that will be published,
        allowing the socket to allocate appropriate buffer sizes for message reception.
        """
        for topic_name in self.publishers:
            # Receive the length of each message from the communication socket
            data = self.communication_socket.recv(4)       
            length = struct.unpack('!i', data)[0] 
            self.message_lens[topic_name] = length
            # Accumulate total message length and update the receive list
            self.msg_len += self.message_lens[topic_name]
            self.receive_list.append(self.receive_list[-1]+self.message_lens[topic_name])

    def send_msg_lengths(self)-> bool:
        """
        Sends the lengths of messages that the socket subscribes to.
        This method communicates the length of each subscribed message to the opposite socket,
        ensuring that the receiver can properly handle incoming data.
        """
        for topic_name in self.subscribers:
            # Get the length of each message
            length = self.message_lens[topic_name]
            # Pack the length into a byte structure and send it
            data = struct.pack('!i', length)
            self.communication_socket.send(data)

    def zero_messages(self, dictionary: Dict) -> None:
        """
        Sets all values of a dictionary to None. This is done to reset the messages 
        after they have been sent, ensuring that the same messages are not sent 
        repeatedly or erroneously.

        Args:
            dictionary (Dict): The dictionary whose values are to be reset.
        """
        # Iterate over each key in the dictionary and set its value to None
        for key in dictionary:
            dictionary[key] = None

    def send_msgs(self)-> None:
        """
        Sends all messages to the other side of the socket.
        This method aggregates all messages to be sent into a single buffer and 
        transmits it over the socket, ensuring efficient and synchronized communication.
        """

        # Initialize an empty buffer to accumulate all messages
        total_buffer = None

        # Iterate over each message to be sent
        for topic_name in self.subscriber_msgs:
            # Prepare a byte array for the message based on its expected length
            byte_array = bytearray(self.message_lens[topic_name])
            msg = self.subscriber_msgs[topic_name]

            # Convert the ROS message to a byte stream
            temp_buffer = self.ros_msg_to_bytes(msg)
            
            # Check if the actual message length exceeds its expected length
            msg_len = len(temp_buffer)
            assert msg_len <= self.message_lens[topic_name], f'{topic_name} was set to have length of {self.message_lens[topic_name]} but has {msg_len} instead. Please correct it in your launch file'

            # Add the current message to the total buffer
            byte_array[:msg_len] = temp_buffer
            if total_buffer is None:
                total_buffer = byte_array
            else:
                total_buffer += byte_array
  
        # Send the aggregated buffer through the socket
        self.communication_socket.send(total_buffer)
        
    def receive_messages(self):
        """
        Receives all messages from the other side of the socket.
        This method accumulates received data into a buffer and then decodes each message
        before publishing it to the respective ROS topic.
        """
        # Initialize a buffer to accumulate received bytes
        msg_bytes = None
        i = 0

        # Continuously receive data until the entire message is received
        while True:
            data = self.communication_socket.recv(self.msg_len)
            if not msg_bytes:
                msg_bytes = data
            else:
                msg_bytes += bytearray(data)
            i += 1
            # Exit the loop when the entire message has been received
            if not data or len(msg_bytes) == self.msg_len: 
                break
        
        # Decode each received message and publish it
        for i, topic_name in enumerate(self.publishers):
            # Determine the start and end points of the current message in the buffer
            start = self.receive_list[i]
            end = self.receive_list[i+1]
            temp_byte = msg_bytes[start:end]
            
            # Deserialize the message from bytes to ROS message format
            msg = self.message_types[topic_name]().deserialize(temp_byte)

            # Publish the message to the corresponding ROS topic
            self.publishers[topic_name].publish(msg)

    def wrap_socket_ssl_context(self, socket: socket.socket, cert_path: str, key_path: str, cert_verify_path: str, is_server: bool):
        """
        Wraps a standard socket into an SSL socket with the appropriate context and settings.
        This method configures and applies SSL settings based on whether the socket is for a server or a client.

        Args:
            socket (socket.socket): The socket to be wrapped.
            cert_path (str): Path to the SSL certificate.
            key_path (str): Path to the SSL private key.
            cert_verify_path (str): Path to the verification certificate.
            is_server (bool): Flag to indicate if the socket is for a server (True) or a client (False).
        """
        # Determine if self-signed certificates are allowed
        allow_self_signed = rospy.get_param('~allow_self_signed', True)

        # Create an SSL context with the appropriate protocol for server or client
        protocol = ssl.PROTOCOL_TLS_SERVER if is_server else ssl.PROTOCOL_TLS_CLIENT
        context = ssl.SSLContext(protocol)
        
        # Configure the SSL context for server or client
        if is_server:
            # Server must require client certificate
            context.verify_mode = ssl.CERT_REQUIRED
        else:
            # Client can optionally bypass hostname verification for self-signed certificates
            context.check_hostname = False if allow_self_signed else True
            context.verify_mode = ssl.CERT_NONE if allow_self_signed else ssl.CERT_REQUIRED
            
        # Load the certificate chain and verification locations
        context.load_cert_chain(cert_path, key_path)
        context.load_verify_locations(cert_verify_path)

        # Attempt to wrap the socket with the SSL context
        try:
            if is_server:
                ssl_socket = context.wrap_socket(socket, server_side=True)
            else:
                ssl_socket = context.wrap_socket(socket, server_hostname=self.hostname) 
        except Exception as e:
            # Raise an error if SSL wrapping fails
            raise ConnectionError(e)
        
        return ssl_socket
    
    @staticmethod
    def ros_msg_to_bytes(msg) -> bytes:
        """
        Serializes a ROS message into bytes.
        This method is used to convert a ROS message into a byte stream, making it suitable 
        for transmission over network sockets or for any other operations that require 
        the message in a byte format.

        Args:
            msg: A ROS message object to be serialized.

        Returns:
            bytes: The serialized byte stream of the ROS message.
        """
        # Initialize a BytesIO buffer to hold the serialized data
        buf = BytesIO()
        # Serialize the ROS message into the buffer
        msg.serialize(buf)
        # Retrieve the byte value of the serialized data
        buf = buf.getvalue()
        return buf

class ROSurgicalClient(ROSurgicalSocket):
    def __init__(self, hostname: str, port: str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        """
        Initializes a ROSurgicalClient instance, which extends ROSurgicalSocket with client-specific functionalities.

        This constructor sets up the socket connection (with or without SSL/TLS), connects to the server, 
        and handles the initial exchange of message lengths.

        Args:
            hostname (str): IP address of the server.
            port (str): Port number for the connection.
            message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path: 
                See ROSurgicalSocket documentation for these parameters.
        """
        # Initialize the parent ROSurgicalSocket class
        super().__init__(hostname, port, message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path)

        # Create a standard socket
        self.socket_no_ssl = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket_no_ssl.setblocking(True)

        # Determine if SSL/TLS wrapping is required
        paths = [cert_path, key_path, cert_verify_path]
        wrap_ssl = all(path is not None for path in paths)

        # Establish SSL/TLS or standard TCP/IP connection
        if wrap_ssl:
            rospy.loginfo("Establishing SSL/TLS connection")
            self.socket = self.wrap_socket_ssl_context(self.socket_no_ssl, cert_path, key_path, cert_verify_path, False)
        else:
            rospy.loginfo("Establishing TCP/IP connection without SSL")
            self.socket = self.socket_no_ssl
        
        # Connect to the server
        self.socket.connect((hostname, port))
        rospy.loginfo(f'Successfully connected {hostname} on port {port}.')
        rospy.sleep(1) # Short delay to ensure the connection is established

        # Exchange message lengths with the server
        self.send_msg_lengths()
        self.receive_msg_lengths()
        rospy.sleep(1) # Short delay after exchanging message lengths

        # Set flags indicating the readiness to send and receive messages   
        self.all_sent = True
        self.all_received = True

        # Start the ROS event loop
        rospy.spin()

    @property
    def communication_socket(self) -> socket.socket:
        """
        Property to access the communication socket of the client.

        Returns:
            socket.socket: The socket object used for communication in the client.
        """
        return self.socket

class ROSurgicalServer(ROSurgicalSocket):
    def __init__(self, hostname: str, port: str, message_types: List[rospy.Message], topic_names: List[str], ros_roles: List[str], cert_path: str, key_path: str, cert_verify_path: str) -> None:
        """
        Initializes a ROSurgicalServer instance, extending ROSurgicalSocket with server-specific functionalities.

        This constructor sets up the server socket (with or without SSL/TLS), binds it to the given hostname and port,
        listens for and accepts a client connection, and manages the initial exchange of message lengths.

        Args:
            hostname (str): The hostname or IP address to bind the server to.
            port (str): The port number to bind the server to.
            message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path: 
                See ROSurgicalSocket documentation for these parameters.
        """
        # Initialize the parent ROSurgicalSocket class
        super().__init__(hostname, port, message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path)

        # Create and configure a standard socket for the server
        self.socket_no_ssl = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.socket_no_ssl.setblocking(True)
        self.socket_no_ssl.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_no_ssl.bind((hostname, port))
        rospy.loginfo(f'Server {hostname} on port {port} launched.')

        # Determine if SSL/TLS wrapping is required
        paths = [cert_path, key_path, cert_verify_path]
        wrap_ssl = all(path is not None for path in paths)

        # Wrap socket with SSL/TLS if requested
        if wrap_ssl:
            rospy.loginfo("Establishing SSL/TLS connection")
            self.socket = self.wrap_socket_ssl_context(self.socket_no_ssl, cert_path, key_path, cert_verify_path, True)
        else:
            rospy.loginfo("Establishing TCP/IP connection without SSL.")
            self.socket = self.socket_no_ssl

        # Listen for incoming client connections
        self.socket.listen(1) # Allow single connection only
        rospy.loginfo(f'Server {hostname} listening to port {port}.')

        # Accept a connection from the client
        self.client_socket, self.client_address = self.socket.accept()
        rospy.loginfo(f'Connection from {self.client_address} has been established.')
        rospy.sleep(2) # Short delay after accepting the connection

        # Exchange message lengths with the client
        self.receive_msg_lengths()
        self.send_msg_lengths()
        rospy.sleep(2)

        # Set flags indicating readiness to send and receive messages      
        self.all_sent = True
        self.all_received = True

        # Start receiving messages from the client
        self.receive_messages()

        # Start the ROS event loop
        rospy.spin()

    @property
    def communication_socket(self) -> socket.socket:
        """
        Property to access the communication socket of the server.

        Returns:
            socket.socket: The socket object used for communication with the client.
        """
        return self.client_socket