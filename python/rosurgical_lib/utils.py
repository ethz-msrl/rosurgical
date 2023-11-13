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

import yaml
import importlib
from typing import Tuple, List

import rospy
from jsk_rviz_plugins.msg import OverlayText

def message_type_factory(msg_type: str) -> rospy.Message:
    """
    Dynamically generates a ROS message object of the specified type.

    This function is useful in scenarios where ROS message types are determined
    at runtime rather than being statically defined. It splits the msg_type
    string to extract the module and class names, imports the module dynamically,
    and retrieves the message class from it.

    Parameters:
    - msg_type (str): A string representing the ROS message type in the format
      'module/MessageClass'. For example, 'std_msgs/String'.

    Returns:
    - rospy.Message: An instance of the specified ROS message type.

    Raises:
    - ImportError: If the module corresponding to the given message type cannot be imported.
    - AttributeError: If the message class is not found in the specified module.
    - ValueError: If the msg_type string format is incorrect.

    Example:
    string_msg = message_type_factory('std_msgs/String')  # Creates a String message object
    """

    try:
        # Split the message type into module name and class name
        module_name, class_name = msg_type.rsplit('/', 1)
        # Dynamically import the module
        module = importlib.import_module(f'{module_name}.msg')

        # Return the message class from the module
        return getattr(module, class_name)
    
    except ImportError as e:
        raise ImportError(f"Could not import module for message type '{msg_type}': {e}")
    except AttributeError as e:
        raise AttributeError(f"Message type '{class_name}' not found in module '{module_name}.msg': {e}")
    except ValueError as e:
        raise ValueError(f"Invalid message type format '{msg_type}': {e}")


def create_latency_overlay_msg(latency: float) -> OverlayText:
    """
    Creates an overlay text message for displaying communication latency in RViz.

    This function generates an OverlayText message, typically used in RViz
    visualizations, to display the communication latency. The latency is
    shown in milliseconds.

    Args:
        latency (float): Communication latency in seconds.

    Returns:
        OverlayText: Overlay text message formatted to display latency.
    """
    # Initialize OverlayText message
    msg = OverlayText()
    
    # Set dimensions and position of the overlay
    msg.width = 200
    msg.height = 50
    msg.left = 0
    msg.top = 0

    # Configure background color (black with some transparency)
    msg.bg_color.r = 0.0
    msg.bg_color.g = 0.0
    msg.bg_color.b = 0.0
    msg.bg_color.a = 0.4  # Transparency

    # Configure foreground color (green)
    msg.fg_color.r = 0.0
    msg.fg_color.g = 1.0
    msg.fg_color.b = 0.0
    msg.fg_color.a = 1.0  # Opaque

    # Set font properties
    msg.line_width = 2
    msg.text_size = 14

    # Set the text to display latency in milliseconds
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
    ros_roles = []
    
    # Get all topic names
    for topic in data:
        # Get message type
        topic_names.append(topic)

        # Get message length
        msg_type = data[topic]["msg_type"]
        message_types.append(msg_type)

        # Get ros role
        origin = data[topic]["origin"]
        assert origin in ["server", "client"]
        if origin == tcp_role:
            ros_role = "subscriber"
        else:
            ros_role = "publisher"
        ros_roles.append(ros_role)

    return (topic_names, message_types, ros_roles)
