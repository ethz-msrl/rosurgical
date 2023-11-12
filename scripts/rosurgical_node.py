#!/usr/bin/env python

import os
import rospy

from rosurgical_lib.secure_socket import ROSurgicalServer, ROSurgicalClient
from rosurgical_lib.utils import parse_topic_paramters


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('rosurgical_node')

    # TCP role
    tcp_role = rospy.get_param('~tcp_role')
    assert tcp_role in ['server', 'client'], 'tcp_role should be either "server" or "client"'

    # Socket parameters
    hostname = rospy.get_param('~hostname')
    port = rospy.get_param('~port')
    if tcp_role == 'server':
        assert port > 1024, 'The port of the server must be above 1024. If you need a port below 1024, you can set a port above 1024 and redirect the desired incoming connection at the desired port to your port locally.'

    # SSL cerificates
    wrap_ssl = rospy.get_param('~wrap_ssl', True)
    if wrap_ssl:
        cert_path = os.path.expanduser(rospy.get_param('~cert_path'))
        key_path = os.path.expanduser(rospy.get_param('~key_path'))
        cert_verify_path = os.path.expanduser(rospy.get_param('~cert_verify_path'))

        for path in [cert_path, key_path, cert_verify_path]:
            assert os.path.exists(path), f"File could not be found: {path}"

    # Get topic details
    yaml_dir = rospy.get_param('~topic_yaml')
    assert os.path.exists(yaml_dir), f"Topic yaml could not be found {yaml_dir}"
    topic_names, message_types, msg_lens, ros_roles = parse_topic_paramters(tcp_role, yaml_dir)

    # Initialize socket
    try:
        if tcp_role == 'server':
            socket = ROSurgicalServer(hostname, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)
        elif tcp_role == 'client':
            socket = ROSurgicalClient(hostname, port, message_types, topic_names, ros_roles, msg_lens, cert_path, key_path, cert_verify_path)
    
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        raise
    
    finally:
        if 'socket' in locals() or 'socket' in globals():
            socket.socket.close()
            rospy.loginfo(f"Closed socket {hostname}:{port}")
