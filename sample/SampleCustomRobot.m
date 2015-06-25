% Creation of a custom-defined robot.  
% This is a sample script to demonstrate some simple functionality of the 
% 3D drawing package for robot visualization.  This script defines a RP 
% (revolute-prismatic) planar robot, and has it follow a simple 
% prescribed path.
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

% Just some useful constants for defining kinematics
x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

%%% Define robot
simple_robot.name = 'sample_robot';
% Kinematics
simple_robot.kin.H = [z0 x0];           % Actuation Axes
simple_robot.kin.P = [zed 2*x0 1*x0];   % Inter-Joint Translation
simple_robot.kin.joint_type = [0 1];    % Joint Ttype: 
                                        %   0 for revolute, 
                                        %   1 for prismatic, 
                                        %   2 for revolute - mobile, 
                                        %   3 for translational - mobile

% Joint limits
simple_robot.limit.lower_joint_limit = [-2*pi 0];
simple_robot.limit.upper_joint_limit = [2*pi 1];


% Visualization constants

% Define each joint visualization
% struct array placeholder for all joints
simple_robot.vis.joints = struct('param',cell(1,2),'props',cell(1,2));

% joint 1 is a revolute joint, define according to createCylinder
simple_robot.vis.joints(1).param = struct('radius', 0.2, 'height', 0.4);
simple_robot.vis.joints(1).props = {'FaceColor', [1;0;0]};

% joint 2 is prismatic, define according to createPrismaticJoint
simple_robot.vis.joints(2).param = struct('width', 0.3, 'length', 0.3, ...
                                          'height', 1, 'sliderscale', 0.8);
simple_robot.vis.joints(2).props = {}; % Keep shape defaults

% Define each link visualization
simple_robot.vis.links = struct('handle', cell(1,3), ...
                                'R', cell(1,3), 't', cell(1,3), ...
                                'param',cell(1,3),'props',cell(1,3));
% Want both links to be cylinders
simple_robot.vis.links(2).handle = @createCylinder;
simple_robot.vis.links(2).R = rot(y0,pi/2);
simple_robot.vis.links(2).t = 0.85*x0;
simple_robot.vis.links(2).param = struct('radius', 0.2, 'height', 1.3);
simple_robot.vis.links(2).props = {'FaceColor', [0;1;0],'EdgeAlpha', 0};

simple_robot.vis.links(3).handle = @createCylinder;
simple_robot.vis.links(3).R = rot(y0,pi/2);
simple_robot.vis.links(3).t = 0.75*x0;
simple_robot.vis.links(3).param = struct('radius', 0.1, 'height', 0.5);
simple_robot.vis.links(3).props = {}; % Keep shape defaults

% Define frames
simple_robot.vis.frame = struct('scale', 0.4, 'width', 0.05);

% Define gripper according to createParallelJawGripper
Rg = rot(y0,pi/2)*rot(z0,pi/2);
tg = sum(simple_robot.kin.P,2);
gripper = struct('aperture',0.4,'height',0.25);

figure(1); clf;
h_robot = createRobot(eye(3), zed ,simple_robot,'CreateFrames','off');
% Create gripper in appropriate orientation
h_gripper = createParallelJawGripper(Rg, zed, gripper);
% Attach gripper to simple robot to get complete robot
h_full_robot = combineRobots(h_robot,h_gripper,'sample_robot');
axis equal;
axis([-4.5 4.5 -4.5 4.5 -1 1]);
view([0 90]);


%% Animate robot with joint displacements

q.names = {'sample_robot', 'gripper_left_jaw', 'gripper_right_jaw'};
q.states = {[0;0], 0, 0};

T = 5; dt = 0.01;
t = 0:dt:T;

q1 = simple_robot.limit.upper_joint_limit(1)*t/T;
q2 = simple_robot.limit.upper_joint_limit(2)*(1 - cos(2*pi*t/T))/2;
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
    q.states = {[q1(k);q2(k)] qg(k) qg(k)};
    h_full_robot = updateRobot(q,h_full_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
