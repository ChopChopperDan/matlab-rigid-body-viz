% Creation and Functionality of a custom-defined robot
% This is a sample script to demonstrate the functionality for the 3D
% drawing package for robot visualization.  In this example, we create a
% Staubli tx40, attach a gripper to it and generate an object within the 
% workspace.  The robot is then sent through a series of motions
% to execute a pick-and-place task to grab the object and position it
% somewhere else.
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

% Just some useful constants for defining kinematics
x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

%%%%%% Define Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Define robot using default define file with space for 3 robots (arm + 
%%%     2 jaws for parallel jaw gripper)
robot = defineEmptyRobot(3);
robot_structure = defineEmptyRobotStructure(3);


% Get predefined constants from robot definitions for Staubli tx40
robot(1) = defineStaublitx40();
robot_structure(1).name = robot(1).name;

% Set gripper parameters and get definition structures
gripper_param = struct('aperture', 0.1, 'height', 0.05, 'fingerwidth', 0.02);
Og = [rot(y0,pi/2)*rot(z0,pi/2) zed; zed' 1];
[robot(2:3), robot_structure(2:3)] = ...
                defineParallelJawGripper(gripper_param);

% Set structure to have the grippers attached the to the end of the tx40
robot_structure(1).right = {robot(2:3).name};
robot_structure(2).left = robot(1).name;
robot_structure(3).left = robot(1).name;

%%%%%% Create Environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
% create robot
h_robot = createCombinedRobot(robot, robot_structure);
axis equal;
axis([-1 1 -1 1 0 1]);

% place object in world
load_param = struct('radius',0.04);
h_load = createEllipsoid(eye(3),[0;0;0],load_param,'FaceColor',[0;1;1]);
h_load = updateRigidBody(eye(3), [0.035;.338;0.04], h_load);
h_load.labels = attachPrefix('load_', h_load.labels);


%% Animate robot with joint displacements

% constant time step during animation
dt = 0.01;

% actuator structure
q.names = {h_robot.robots.name};
q.states = {zeros(6,1), 0, 0};

% pre-defined robot motion
t = 0:dt:1;
q_arm = zeros(6, numel(t));
q_arm(2,:) = -80*pi/180*t;
q_arm(3,:) = -80*pi/180*t;

% Move robot end effector towards object
for k=1:length(t)
    tic;
    q.states{1} = q_arm(:,k);
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
pause(5*dt);

% grab object
% Animate gripper closing
t = 0:dt:0.2;
qg = 1/2*(gripper_param.aperture - 2*load_param.radius)*t/t(end);
for k=1:length(t)
    tic;
    q.states{2} = qg(k);
    q.states{3} = qg(k);
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end

h_robot = graspLoad(h_load,h_robot, h_robot.robots(1).name);
pause(5*dt);

% pre-defined robot motion
t = 0:dt:2;
q_arm = zeros(6, numel(t));
q_arm(2,:) = -80*pi/180*(t(end)/2 - t);
q_arm(3,:) = -80*pi/180*(t(end)/2 - t);

% move robot to desired new location
for k=1:length(t)
    tic;
    q.states{1} = q_arm(:,k);
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
pause(5*dt);

% Animate gripper opening
t = 0:dt:0.2;
qg = 1/2*(gripper_param.aperture - 2*load_param.radius)*(1 - t/t(end));
for k=1:length(t)
    tic;
    q.states{2} = qg(k);
    q.states{3} = qg(k);
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end

[h_robot, h_load] = releaseLoad(h_robot, h_robot.robots(1).name);
pause(5*dt);

% return robot to zero position
t = 0:dt:1;
q_arm = zeros(6, numel(t));
q_arm(2,:) = 80*pi/180*(1 - t/t(end));
q_arm(3,:) = 80*pi/180*(1 - t/t(end));

for k=1:length(t)
    tic;
    q.states{1} = q_arm(:,k);
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
