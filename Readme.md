# rosurgical

This package is used for TCP communication between two system. In particular, it was implemented with Telesurgery in mind but is not limited to it. 

## Launch files (examples for telesurgery)

* **`navion_rviz_child.launch`** Launch file to be started on the child (Navion) computer. This file starts all nodes relevant for controlling the Navion and starts a TCP Socket for each message that needs transfered between the parent and the child computer.

* **`navion_rviz_remote.launch`** Launch file to be started on the parent (remote) computer. This file starts all nodes relevant for the input controllers and visualizations and tries to connect to the corresponding sockets on the host computer for each topic.
