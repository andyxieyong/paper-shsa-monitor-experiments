# Demo Commands

Uses custom [docker images] for demos with [SHSA] and [Daisy].

The commands below demonstrate [shsa-prolog] for the [SASO 2019](https://saso2019.cs.umu.se/) paper:
*"Fault Detection exploiting Implicit Redundancy with Uncertainties in Space and Time"*


## Network

Notebook has to be time-synchronized with [Daisy] (because the demo uses transforms).
Used router to connect to the hosts on the rover
(but NTP could be established with IP forwarding too).


## Application

**Create a map before you start the demo.**

Start application, [Daisy] starts moving around (turning when it comes near an obstacle):
```bash
$ docker run --rm -it --network=host \
    -v /home/denise/.ssh/:/root/.ssh/ \
    -v /home/denise/ws/ros/shsa/src/paper-shsa-monitor-experiments/:/catkin_ws/src/demo/ \
    ros:daisy roslaunch demo demo.launch notebook:=<your hostname>
```

Enable motors (if necessary):
```bash
$ docker run --rm -it --network=host ros:daisy roslaunch shsa_ros enablemotors.launch
```

Set initial pose for SLAM (amcl node) via rviz.
```bash
$ x11docker --hostnet --home ros:gui rviz
```
To reuse configs for rviz use option `--home` of `x11docker`.

### Switch between tele-operation and wandering

Select tele-operation (default `/teleop/cmd_vel` used):

```bash
$ docker run --rm -it --network=host ros:daisy rosrun topic_tools mux_select mux_cmdvel /teleop/cmd_vel
```

Or wanderer:

```bash
$ docker run --rm -it --network=host ros:daisy rosrun topic_tools mux_select mux_cmdvel /wanderer/cmd_vel
```


## SHSA

Start monitoring:
```bash
$ docker run --rm -it --network=host \
    -v /home/denise/ws/ros/shsa/src/saso2019/ \
    ros:daisy roslaunch shsa_ros monitor.launch \
    modelfile:=/catkin_ws/src/demo/config/dmin.pl
```

Attack the laser scanner used to avoid collisions:
```bash
$ docker run --rm -it --network=host ros:daisy roslaunch shsa_ros attack.launch
```


## Visualization

```bash
$ x11docker --hostnet ros:gui rqt_plot /emergency_stop/dmin/data /dmin_monitor/value_0/data /dmin_monitor/value_1/data
$ x11docker --hostnet ros:gui rqt_graph
$ x11docker --hostnet --home ros:gui rviz
```


## Log

```bash
$ docker run --rm -it --network=host \
    -v /home/denise/ws/ros/shsa/src/paper-shsa-monitor-experiments/:/catkin_ws/src/demo/ \
    ros:daisy bash
# rosbag record -o /catkin_ws/src/demo/log/topics /cmd_vel /emergency_stop/dmin
```
Unfortunately, docker doesn't wait until rosbag finishes when pressing `Ctrl-C`.
So log over the `bash` in a container.


## Create a Map

```bash
$ docker run --rm -it --network=host \
    -v /home/denise/.ssh/:/root/.ssh/ \
    -v /home/denise/ws/ros/shsa/src/paper-shsa-monitor-experiments/:/catkin_ws/src/demo/ \
    ros:daisy roslaunch demo create_map.launch notebook:=<your hostname>
```

Visualize map:
```bash
$ x11docker --hostnet --home ros:gui rviz
```
When rviz is running with the `--home` option
you can copy the config from `config/create_map.rviz` to `~/x11docker/ros/.rviz/`
so you can open it in rviz.

Save map:
```bash
$ docker run --rm -it --network=host \
    -v /home/denise/ws/ros/shsa/src/paper-shsa-monitor-experiments/:/demo/
    ros:daisy rosrun map_server map_saver -f /demo/config/map
```
Adapt the image path in `map.yaml` to `./map.pgm`.


[docker images]: https://github.com/dratasich/docker
[Daisy]: https://tuw-cpsg.github.io/tutorials/daisy/
[SHSA]: https://github.com/dratasich/shsa_ros
[shsa-prolog]: https://github.com/dratasich/shsa-prolog
