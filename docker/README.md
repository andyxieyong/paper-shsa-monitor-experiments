Dockerfile
==========

Custom image for the experiments in the shsa-prolog paper.

Note, this repository is not part of the Docker file!
Clone the repository separately and mount it to the docker container.


Build
-----

Build the docker image (see `docker-build(1)`):
```bash
$ cd /path/to/paper-shsa-monitor-experiments/docker
$ docker build -t paper:shsa-prolog .
```
