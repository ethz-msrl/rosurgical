import yaml
import importlib
from typing import Tuple, List

import rospy
from jsk_rviz_plugins.msg import OverlayText

def message_type_factory(msg_type: str) -> rospy.Message:
    try:
        module_name, class_name = msg_type.rsplit('/', 1)
        module = importlib.import_module(f'{module_name}.msg')
        return getattr(module, class_name)
    except ImportError as e:
        raise ImportError(f"Could not import module for message type '{msg_type}': {e}")
    except AttributeError as e:
        raise AttributeError(f"Message type '{class_name}' not found in module '{module_name}.msg': {e}")
    except ValueError as e:
        raise ValueError(f"Invalid message type format '{msg_type}': {e}")


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
        assert origin in ["server", "client"]
        if origin == tcp_role:
            ros_role = "subscriber"
        else:
            ros_role = "publisher"
        ros_roles.append(ros_role)

    return (topic_names, message_types, message_lengths, ros_roles)
