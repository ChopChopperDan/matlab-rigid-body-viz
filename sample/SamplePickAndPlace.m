% Creation and Functionality of a custom-defined robot
% This is a sample script to demonstrate the functionality for the 3D
% drawing package for robot visualization.  In this example, we create a
% custom robot and an object within the workspace.  The robot is then told
% to execute a pick-and-place motion to grab the object and position it
% somewhere else.

clear all; close all;


% Add path to drawing library if not already included
if isempty(strfind(path, 'matlab-rigid-body-viz'))
    addpath('../../matlab-rigid-body-viz/src');
end

x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

%%%%%% Define Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematics
simple_robot.H = [z0 z0];
simple_robot.P = [zed 2*x0 1*x0];
simple_robot.type = [0 0];
simple_robot.n = 2;
simple_robot.origin = [eye(3) zed; zed' 1];

% link type: 0 for no link
%            1 for cylindrical link
%            2 for cuboid link
simple_robot.link_type = [0 1 1];

% link dimensions
simple_robot.link(2).radius = 0.2;
simple_robot.link(2).height = 1.6;
simple_robot.link(2).R0 = rot(y0,pi/2);
simple_robot.link(2).t0 = 1*x0;
simple_robot.link(2).props = {};

simple_robot.link(3).radius = 0.15;
simple_robot.link(3).height = 0.8;
simple_robot.link(3).R0 = rot(y0,pi/2);
simple_robot.link(3).t0 = 0.6*x0;
simple_robot.link(3).props = {};

simple_robot.joint(1).radius = 0.2;
simple_robot.joint(1).height = 0.4;
simple_robot.joint(1).props = {};
simple_robot.joint(2).radius = 0.2;
simple_robot.joint(2).height = 0.4;
simple_robot.joint(2).props = {};

simple_robot.frame.scale = 0.4;
simple_robot.frame.width = 0.05;

simple_robot.gripper.aperture = 0.4;
simple_robot.gripper.stroke = 0.4;
simple_robot.gripper.height = 0.25;
simple_robot.gripper.R0 = rot(y0,pi/2)*rot(z0,pi/2);

%%%%%% Create Environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
h_robot = createRobot(simple_robot,'CreateFrames','on');
axis equal;
axis([-3.5 3.5 -3.5 3.5 -1 1]);
view([0 90])

% place object in world
load_param.radius = 0.19;
load = createEllipsoid(eye(3),[0;3.25;0],load_param,'FaceColor',[0;1;1]);
load.labels = attachPrefix('load_', load.labels);


%% Animate robot with joint displacements

% timestepping
t = 0:0.02:1;

% pre-defined robot motion
q1 = pi/2*t;
q2 = 0*t;

% Move robot end effector towards object
for k=1:length(t)
    h_robot = updateRobot([q1(k);q2(k)],h_robot);
    drawnow;
end

% grab object
if norm(h_robot.frame(end).t - load.t) <= 0.5
    h_robot = graspLoad(load,h_robot);
end

% pre-defined robot motion
q1 = q1(end)-pi*t;
q2 = q2(end)-3*pi/4*t;

% move robot to desired new location
for k=1:length(t)
    h_robot = updateRobot([q1(k);q2(k)],h_robot);
    drawnow;
end

% release object
[h_robot, load] = releaseLoad(h_robot);

% return robot to zero position
q1 = q1(end) - q1(end)*t;
q2 = q2(end) - q2(end)*t;

for k=1:length(t)
    h_robot = updateRobot([q1(k);q2(k)],h_robot);
    drawnow;
end
