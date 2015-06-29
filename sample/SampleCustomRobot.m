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

%%% Define robot using default define file.  Want 3 robots to hold the arm
%%% and both jaws of a parallel jaw gripper
sample_robot =  defineEmptyRobot(3);

% Define gripper according to parameterization of ParallelJawGripper
Og = [rot(y0,pi/2)*rot(z0,-pi/2) zed; zed' 1];
gripper_param = struct('aperture',0.4,'height',0.25);
[gripper_const, gripper_structure] = ...
                defineParallelJawGripper(gripper_param, 'Origin', Og);
sample_robot(2:3) = gripper_const;

sample_robot(1).name = 'sample_robot';
% Kinematics
sample_robot(1).kin.H = [z0 x0];           % Actuation Axes
sample_robot(1).kin.P = [zed 2*x0 1*x0];   % Inter-Joint Translation
sample_robot(1).kin.joint_type = [0 1];    % Joint Ttype: 
                                           %   0 for revolute, 
                                           %   1 for prismatic, 
                                           %   2 for revolute - mobile, 
                                           %   3 for translational - mobile

% Joint limits
sample_robot(1).limit.lower_joint_limit = [-2*pi 0];
sample_robot(1).limit.upper_joint_limit = [2*pi 1];


% Visualization constants

% Define each joint visualization
% struct array placeholder for all joints
sample_robot(1).vis.joints = struct('param',cell(1,2),'props',cell(1,2));

% joint 1 is a revolute joint, define according to createCylinder
sample_robot(1).vis.joints(1).param = struct('radius', 0.2, 'height', 0.4);
sample_robot(1).vis.joints(1).props = {'FaceColor', [1;0;0]};

% joint 2 is prismatic, define according to createPrismaticJoint
sample_robot(1).vis.joints(2).param = struct('width', 0.3, ...
                                            'length', 0.3, ...
                                            'height', 1, ...
                                            'sliderscale', 0.8);
sample_robot(1).vis.joints(2).props = {}; % Keep shape defaults

% Define each link visualization
sample_robot(1).vis.links = struct('handle', cell(1,3), ...
                                'R', cell(1,3), 't', cell(1,3), ...
                                'param',cell(1,3),'props',cell(1,3));
% Want both links to be cylinders
sample_robot(1).vis.links(2).handle = @createCylinder;
sample_robot(1).vis.links(2).R = rot(y0,pi/2);
sample_robot(1).vis.links(2).t = 0.85*x0;
sample_robot(1).vis.links(2).param = struct('radius', 0.2, 'height', 1.3);
sample_robot(1).vis.links(2).props = {'FaceColor', [0;1;0],'EdgeAlpha', 0};

sample_robot(1).vis.links(3).handle = @createCylinder;
sample_robot(1).vis.links(3).R = rot(y0,pi/2);
sample_robot(1).vis.links(3).t = 0.75*x0;
sample_robot(1).vis.links(3).param = struct('radius', 0.1, 'height', 0.5);
sample_robot(1).vis.links(3).props = {}; % Keep shape defaults

% Define frames
sample_robot(1).vis.frame = struct('scale', 0.4, 'width', 0.05);



%%% Define structure for full combined robot
simple_robot_structure = defineEmptyRobotStructure(3);
simple_robot_structure(1:2) = gripper_structure;
simple_robot_structure(1).left = sample_robot(1).name;
simple_robot_structure(2).left = sample_robot(1).name;

simple_robot_structure(3).name = sample_robot.name;
simple_robot_structure(3).right = {gripper_const.name};

% Finally create full robot with both arm and gripper combined into a
% single robot structure
figure(1); clf;
h_sample_robot = createCombinedRobot(sample_robot, simple_robot_structure);
axis equal;
axis([-4.5 4.5 -4.5 4.5 -1 1]);
view([0 90]);



%% Animate robot with joint displacements

% Time sequence
T = 5; dt = 0.01;
t = 0:dt:T;

% Pre-allocate angle structure
q.names = {h_sample_robot.robots.name};
q.states = {[0;0], 0, 0};

% Define paths for each actuator
q1 = sample_robot(1).limit.upper_joint_limit(1) * t/T;
q2 = sample_robot(1).limit.upper_joint_limit(2) * (1 - cos(2*pi*t/T))/2;
qg = sample_robot(2).limit.upper_joint_limit * [t(1:(end-1)/2)/(T/2) ...
                                            (1 - t(1:(end+1)/2)/(T/2))];

% Step through time sequence and update robot at each time instance
for k=1:length(t)
    tic;
    q.states = {[q1(k);q2(k)] qg(k) qg(k)};
    h_sample_robot = updateRobot(q,h_sample_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
