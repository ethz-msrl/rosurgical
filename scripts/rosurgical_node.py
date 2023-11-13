#!/usr/bin/env python

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

import os
import rospy

from rosurgical_lib.secure_socket import ROSurgicalServer, ROSurgicalClient
from rosurgical_lib.utils import parse_topic_parameters


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
    topic_names, message_types, ros_roles = parse_topic_parameters(tcp_role, yaml_dir)

    # Initialize socket
    try:
        if tcp_role == 'server':
            socket = ROSurgicalServer(hostname, port, message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path)
        elif tcp_role == 'client':
            socket = ROSurgicalClient(hostname, port, message_types, topic_names, ros_roles, cert_path, key_path, cert_verify_path)
    
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        raise
    
    finally:
        if 'socket' in locals() or 'socket' in globals():
            socket.socket.close()
            rospy.loginfo(f"Closed socket {hostname}:{port}")
