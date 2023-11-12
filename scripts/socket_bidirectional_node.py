#!/usr/bin/env python
import os
import rospy

from sockets.socket_bidirectional import BidirectionalClient, BidirectionalHost
from sockets.utils import parse_topic_paramters

"""
ToDo:
- Detect bandwidth (adjust video compression) dynamically
- Implement Service calls in a generic manner
"""

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('tcp_socket')

    # Socket parameters
    host = rospy.get_param('~host', None)
    port = rospy.get_param('~port', None)
    assert (port >= 3000 and port <= 3005) or port==443, 'Port should be either 443 or between 3000 and 3005. These ports are opened for the Navion.'

    # TCP role
    tcp_role = rospy.get_param('~tcp_role', None)
    assert tcp_role in ['host', 'client'], 'tcp_role should be either "host" or "client"'

    # SSL cerificates
    cert_path = rospy.get_param('~cert_path', None)
    key_path = rospy.get_param('~key_path', None)
    cert_verify_path = rospy.get_param('~cert_verify_path', None)
    paths = [cert_path, key_path, cert_verify_path]
    assert all(path is None for path in paths) or all(path is not None for path in paths), "Either all paths should be None or no path should be None."
    if all(path is not None for path in paths):
        cert_path = os.path.expanduser(cert_path)
        key_path = os.path.expanduser(key_path)
        cert_verify_path = os.path.expanduser(cert_verify_path)
        assert os.path.exists(cert_path), f"Certificate could not be found {cert_path}"
        assert os.path.exists(key_path), f"Key could not be found {key_path}"
        assert os.path.exists(cert_verify_path), f"Verification certificate could not be found {cert_verify_path}"

    # Get topic details
    yaml_dir = rospy.get_param('~topic_yaml', None)
    topic_names, message_types, msg_lens, ros_roles = parse_topic_paramters(tcp_role, yaml_dir)

    # Initialize socket
    try:
        if tcp_role == 'host':
            socket = BidirectionalHost(host, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)
        elif tcp_role == 'client':
            socket = BidirectionalClient(host, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)
    except Exception as e:
        raise Exception(e)
    finally:
        socket.socket.close(), f"Closing socket {host, port}"
