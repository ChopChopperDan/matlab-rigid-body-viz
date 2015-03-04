% Creation of a custom-defined robot.  
% This is a sample script to demonstrate some simple functionality of the 
% 3D drawing package for robot visualization.  This script defines a RP 
% (revolute-prismatic) planar robot, and has it follow a simple 
% prescribed path.
%
% Be sure to use rigidbodyviz_setup() before running

clear all; close all;

x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

% Kinematics
simple_robot.H = [z0 x0];
simple_robot.P = [zed 2*x0 1*x0];
% joint type: 0 for revolute, 1 for prismatic, 
%             2 for revolute - mobile, 3 for translational - mobile
simple_robot.joint_type = [0 1];
simple_robot.name = 'sample_robot';

% link type: 0 for no link
%            1 for cylindrical link
%            2 for cuboid link
simple_robot.link_type = [0 1 1];

link_props = {'FaceColor', [0;1;0],'EdgeAlpha', 0};
joint_props = {'FaceColor', [1;0;0]};

% link dimensions
simple_robot.links(2).radius = 0.2;
simple_robot.links(2).height = 1.3;
simple_robot.links(2).R0 = rot(y0,pi/2);
simple_robot.links(2).t0 = 0.85*x0;
simple_robot.links(2).props = link_props;

simple_robot.links(3).radius = 0.1;
simple_robot.links(3).height = 0.5;
simple_robot.links(3).R0 = rot(y0,pi/2);
simple_robot.links(3).t0 = 0.75*x0;
simple_robot.links(3).props = {}; % Keep shape defaults

simple_robot.joints(1).radius = 0.2;
simple_robot.joints(1).height = 0.4;
simple_robot.joints(1).props = joint_props;

simple_robot.joints(2).width = 0.3;
simple_robot.joints(2).length = 0.3;
simple_robot.joints(2).height = 1;
simple_robot.joints(2).sliderscale = 0.8;
simple_robot.joints(2).props = {}; % Keep shape defaults

% Define frames
simple_robot.frame.scale = 0.4;
simple_robot.frame.width = 0.05;

% Define gripper
gripper = struct('aperture',0.4,'height',0.25);

figure(1); clf;
h_robot = createRobot(eye(3), zed ,simple_robot,'CreateFrames','off');
% Create gripper in appropriate orientation
h_gripper = createParallelJawGripper(rot(y0,pi/2)*rot(z0,pi/2), zed, ...
                                                    gripper);
% Move into alignment with end effector
h_gripper = updateRigidBody(eye(3),sum(simple_robot.P,2),h_gripper);
% Attach gripper to simple robot to get complete robot
h_full_robot = combineRobots(h_robot,h_gripper);
axis equal;
axis([-4.5 4.5 -4.5 4.5 -1 1]);
view([0 90]);


%% Animate robot with joint displacements

T = 5; dt = 0.01;
t = 0:dt:T;

q1 = 2*pi*t/T;
q2 = simple_robot.joints(2).height*(1 - cos(2*pi*t/T))/2;
if mod(length(t),2) == 0
    qg = gripper.aperture/2*t(1:end/2)/T/2;
    qg = [qg gripper.aperture/2*(1 - t(1:end/2)/(T/2))];
else
    qg = gripper.aperture/2*t(1:(end-1)/2)/(T/2);
    qg = [qg gripper.aperture/2*(1 - t(1:(end+1)/2)/(T/2))];
end
timestamps = zeros(1,length(t));

for k=1:length(t)
    tic;
    q = {{[q1(k);q2(k)];qg(k);qg(k)}};
    h_full_robot = updateRobot(q,h_full_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
