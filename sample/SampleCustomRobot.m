% Creation of a custom-defined robot.  
% This is a sample script to demonstrate some simple functionality of the 
% 3D drawing package for robot visualization.  This script defines a RP 
% (revolute-prismatic) planar robot, and has it follow a simple 
% prescribed path.

clear all; close all;

% Add path to drawing library if not already included
if isempty(strfind(path, 'matlab-rigid-body-viz'))
    addpath('../../matlab-rigid-body-viz/src');
end

x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

% Kinematics
simple_robot.H = [z0 x0];
simple_robot.P = [zed 2*x0 1*x0];
% joint type: 0 for revolute, 1 for prismatic, 
%             2 for revolute - mobile, 3 for translational - mobile
simple_robot.type = [0 1];
simple_robot.n = 2;
simple_robot.origin = [eye(3) zed; zed' 1];

% link type: 0 for no link
%            1 for cylindrical link
%            2 for cuboid link
simple_robot.link_type = [0 1 1];

link_props = {'FaceColor', [0;1;0],'EdgeAlpha', 0};
joint_props = {'FaceColor', [1;0;0]};

% link dimensions
simple_robot.link(2).radius = 0.2;
simple_robot.link(2).height = 1.3;
simple_robot.link(2).R0 = rot(y0,pi/2);
simple_robot.link(2).t0 = 0.85*x0;
simple_robot.link(2).props = link_props;

simple_robot.link(3).radius = 0.1;
simple_robot.link(3).height = 0.5;
simple_robot.link(3).R0 = rot(y0,pi/2);
simple_robot.link(3).t0 = 0.75*x0;
simple_robot.link(3).props = {}; % Keep shape defaults

simple_robot.joint(1).radius = 0.2;
simple_robot.joint(1).height = 0.4;
simple_robot.joint(1).props = joint_props;

simple_robot.joint(2).width = 0.3;
simple_robot.joint(2).length = 0.3;
simple_robot.joint(2).height = 1;
simple_robot.joint(2).sliderscale = 0.8;
simple_robot.joint(2).props = {}; % Keep shape defaults

% Define frames
simple_robot.frame.scale = 0.4;
simple_robot.frame.width = 0.05;

% Define gripper
simple_robot.gripper.width = 0.4;
simple_robot.gripper.height = 0.25;
simple_robot.gripper.R0 = rot(y0,pi/2)*rot(z0,pi/2);

figure(1);
h_robot = createRobot(simple_robot,'CreateFrames','on');
axis equal;
axis([-3.5 3.5 -3.5 3.5 -1 1]);


%% Animate robot with joint displacements

t = 0:0.005:1;

q1 = 2*pi*t;
q2 = simple_robot.joint(2).height*(1 - cos(2*pi*t))/2;
timestamps = zeros(1,length(t));

for k=1:length(t)
    h_robot = updateRobot([q1(k);q2(k)],h_robot);
    drawnow;
end
