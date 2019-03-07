%
% SHSA Knowledge Base for Daisy's dmin Calculation
%
% Denise Ratasich
% 2019-02-26
%

:- use_module(library(shsa)).


% SHSA knowledge base from
% Ratasich et al.: Fault detection. 2019.
%

% dmin
function(dmin, r1, [d_2d]).
function(d_2d, r2, [d_3d]).
function(d_2d, r3, [map, pose]).
function(dmin, r4, [dmin_last, speed]).
function(dmin_last, r5, [dmin]).
% ROS msg types of distance data different -> separate variables
%% function(d_2d, rlaser, laserscan)
%% function(d_2d, rsonar, sonararray)
%% function(d_3d, rpointcloud2, pointcloud2)

% to create executable substitutions: define implementations of the relations
implementation(r1, "dmin.v = min(d_2d.v)").
implementation(r2, "
# row width of depth image
w = 320
# take 100th row (about the height of lidar scan)
h = 100
d_2d.v = [d for i, d in enumerate(d_3d.v) if i >= h*w and i < (h+1)*w]
").
implementation(r3, "
# https://answers.ros.org/question/227400/nearest-obstacle-in-map/
# http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
# TODO
").
implementation(r4, "").
implementation(r5, "").
% helpers: convert ROS msg types
%% implementation(rlaser, "d_2d.v = list(laserscan.v.ranges)").
%% implementation(rsonar, "d_2d.v = list(sonararray.v.ranges)").
%% implementation(rsonar, "d_3d.v = list(image.v.data)").

% hard-code provided itoms
% (though the itoms can be extracted by calling `rostopic list`,
% a mapping to the variables is needed, so we just statically define it here)
itomsOf(dmin, ["/emergency_stop/dmin/data"]).
itomsOf(d_2d, ["/scan/ranges", "/p2os/sonar/ranges"]).
itomsOf(d_3d, ["/tof_camera/frame/depth"]).
