Plot
====

Uses custom [docker image] to make use of [rosbag_pandas](https://pypi.org/project/rosbag_pandas/).

Monitor Data
------------

```bash
$ x11docker --homedir /path/to/paper-shsa-monitor-experiments/ \
    ros:pandas ~/plot/plt_monitor.py ~/log/a.bag
```


[docker image]: https://github.com/dratasich/docker/tree/master/ros-bag-pandas
