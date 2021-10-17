# rviz-detections
ROS code for publishing data in visualizable format. Targeted for ROS melodic.
Has been containerized, so this package should be easy to utilize. Build and
launch commands have been specified in the `Makefile`.


## Usage
Use detection messages, specified by this pacakge, in your project. Also, update the `launch/rviz_detections.launch` to suit your needs.

Then, build this package with `make docker-build` command. Start the *roscore* and *rviz* instances locally. Finally, launch visualization node inside a container with `make docker-launch` command.

### Details
- Container uses host-network, which allows the application to communicate with local
*roscore* and local *rviz* instances. (We don't want to run these inside the container).

- Launch file is mounted inside the container, so configuration parameters can be changed easily.
If the container is already running, then you need to only relaunch it after updating parameters
in the configuration file.

### Debugging
Turn on the `debug` option in the launch file and change `-d` to `-it` in Makefile's launch command,
so that debug prints become visible in the terminal.

### References
Here is good info about markers:  
http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html  
http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_List_.28LINE_LIST.3D5.29
