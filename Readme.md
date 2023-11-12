# rosurgical

`rosurgical` is a ROS package designed to facilitate TCP/IP communication between two systems in a telesurgical setting. It provides a convenient way to exchange ROS topics between programs running on different systems. The desired topics can be defined in configuration file.  

## General Information
`rosurgical` runs two nodes - one on each side of the telesurgery setup. Each socket subscribes to the topics that are defined in a configuration file and originate on the respective host. When a topic is received, it is serialized and transmitted to the opposite host, where is is deserialized and published to the local ROS environment. 

![Block Diagram](misc/block_diagram.png)

`rosurgical` allows offers secure communication between the server and the client, by establishing an SSL/TLS connection. The host authenticate each other and then establish an encrypted connection. 

Additional, the `rosurgical` node on each host publishes the communication latency to its respective ROS environment.

## Node

### `rosurgical_node.py`
This node start a TCP/IP socket and connects to the counterpart. 

#### Parameters
- `~host`: The hostname of the server
- `~port`: The port for the TCP/IP connection
- `~tcp_role`: role of the socket. Can be either `server` or `client`
- `~topic_yaml`: Path to the topic file
- `~cert_path`: Path to the SSL certificate of the own host
- `~key_path`: Path to the private key of the own host 
- `~cert_verify_path`: Path to the SSL certificate of the opposite host for verification

#### Subscribed topics
The node subscribes to the topics defined in the topic yaml files that originate on the own host

#### Published topics
The node publishes the topics defined in the topic yaml files that originate on the opposite host. Furthermore, it publishes two more topics:

- `communication_latency_float` [std_msgs/Float32]: Communication latency in ms
- `communication_latency` [jsk_rviz_plugins/OverlayText]: Communication latency in ms for visulaization in RViz


## Citation

```
@inproceedings{Heemeyer2024,
  doi = {TBD},
  url = {TBD},
  year = {2024},
  month = jun,
  publisher = {{IEEE}},
  author = {Florian Heemeyer and Quentin Boehler and Fabio Leuenberger and Bradley J. Nelson},
  title = {ROSurgical: An Open-Source Framework for Telesurgery},
  booktitle = {2024 International Symposium on Medical Robotics ({ISMR})}
}
```