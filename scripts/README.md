Scripts
=======

Plot logged bag file
--------------------

```bash
$ x11docker --homedir /path/to/paper-shsa-monitor-experiments/ \
    paper:shsa-prolog ~/scripts/plt_log.py ~/log/a.bag
```

Fault injection to logged bag file
----------------------------------

Convert logged ROS data (bag file created with log.launch)
to a CSV file that can be fed to a monitor.
Additional options provide means to inject faults or to introduce/vary signal delays, see:
```bash
$ docker --rm -it -v /path/to/paper-shsa-monitor-experiments/:/root/ \
    paper:shsa-prolog ~/scripts/bag2csv.py -h
```

Run monitor stand-alone (without ROS)
-------------------------------------

```bash
$ docker --rm -it -v /path/to/paper-shsa-monitor-experiments/:/root/ \
    paper:shsa-prolog ~/scripts/run.py -h
:
```
