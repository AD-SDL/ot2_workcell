# OT2 abstraction requirments
Moving OT2 abstractions out of ROS network. 

## Naming Conventions
Package **must** be named `ot2_driver_pkg`, and file must be named `ot2_driver`. **No other package** will be imported into the ROS network, and thus invisible to the ROS layer.  

Packages **must** follow ROS 2 python package format, see useful commands below! (Note, the way you link internal packges **must** also follow ROS 2 package formats)

## Functions needed 
* load_protocol(protocol_path, robot_id)
    * Loads the protocol to the ot2_driver (knows what protocol code is, current driver does this via database)
    * Inserts in error handling into the protocol
* run_protocol(protocol_id, username, ip, port)
    * Runs the given protocol id on the specified internal RPI4
    * TODO: get that internal RPI4 specifications from config file not passed via arguments

## Useful ROS Commands
* `ros2 pkg create --build-type ament_python <package_name>` Creates the barebones for a new ROS 2 package
* `colcon graph --dot | dot -Tpng -o deps.png` Generates a dependency graph

## Installation Instructions
1. 'git clone https://github.com/AD-SDL/ot2_driver_pkg.git`
