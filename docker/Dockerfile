FROM ros:kinetic

SHELL ["/bin/bash", "-c"]

# ROS, python data manipulation and plotting
RUN apt-get update && apt-get install -y \
        ros-kinetic-p2os-msgs \
        ros-kinetic-tf2-ros \
        ros-kinetic-gmapping \
        ros-kinetic-navigation \
        ros-kinetic-image-transport \
        python-pip \
        python-matplotlib \
        python-pandas \
        git

# additional python packages (not in ubuntu repository)
RUN pip install \
        flatdict

# python ws: shsa-prolog
RUN mkdir -p /python_ws
WORKDIR /python_ws
# shsa-prolog source
ADD https://api.github.com/repos/dratasich/shsa-prolog/compare/master...HEAD /dev/null
RUN git clone https://github.com/dratasich/shsa-prolog.git
# install dependencies
WORKDIR /python_ws/shsa-prolog
RUN pip install -r requirements.txt

# ROS packages
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
# Daisy and SHSA
ADD https://api.github.com/repos/tuw-cpsg/general-ros-modules/compare/master...HEAD /dev/null
RUN git clone https://github.com/tuw-cpsg/general-ros-modules.git
ADD https://api.github.com/repos/dratasich/shsa_ros/compare/master...HEAD /dev/null
RUN git clone https://github.com/dratasich/shsa_ros.git
ADD https://api.github.com/repos/tuw-cpsg/tof-ros-interface/compare/master...HEAD /dev/null
RUN git clone https://github.com/tuw-cpsg/tof-ros-interface.git
# rosbag_pandas and dependencies
ADD https://api.github.com/repos/eurogroep/rosbag_pandas/compare/master...HEAD /dev/null
RUN git clone https://github.com/eurogroep/rosbag_pandas.git
ADD https://api.github.com/repos/uos/rospy_message_converter/compare/master...HEAD /dev/null
RUN git clone https://github.com/uos/rospy_message_converter.git
# # this demo repo for the experiments
# ADD https://api.github.com/repos/tuw-cpsg/paper-shsa-monitor-experiments/compare/master...HEAD /dev/null
# RUN git clone https://github.com/tuw-cpsg/paper-shsa-monitor-experiments.git

# build custom ROS packages
WORKDIR /catkin_ws
RUN source /opt/ros/kinetic/setup.bash && catkin_make

# initialize ROS (master uri, environments, etc.)
COPY docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]

# default command
CMD ["bash"]
