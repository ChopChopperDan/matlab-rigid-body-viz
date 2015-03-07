% Creation and Functionality of a custom-defined robot
% This is a sample script to demonstrate the functionality for the 3D
% drawing package for robot visualization.  In this example, we create a
% custom robot and an object within the workspace.  The robot is then told
% to execute a pick-and-place motion to grab the object and position it
% somewhere else.
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

%%%%%% Define Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematics
simple_robot.H = [z0 z0];
simple_robot.P = [zed 2*x0 1*x0];
simple_robot.joint_type = [0 0];
simple_robot.name = 'sample_robot';

% link type: 0 for no link
%            1 for cylindrical link
%            2 for cuboid link
simple_robot.link_type = [0 1 1];

% link dimensions
simple_robot.links(2).radius = 0.2;
simple_robot.links(2).height = 1.6;
simple_robot.links(2).R0 = rot(y0,pi/2);
simple_robot.links(2).t0 = 1*x0;
simple_robot.links(2).props = {};

simple_robot.links(3).radius = 0.15;
simple_robot.links(3).height = 0.8;
simple_robot.links(3).R0 = rot(y0,pi/2);
simple_robot.links(3).t0 = 0.6*x0;
simple_robot.links(3).props = {};

simple_robot.joints(1).radius = 0.2;
simple_robot.joints(1).height = 0.4;
simple_robot.joints(1).props = {};
simple_robot.joints(2).radius = 0.2;
simple_robot.joints(2).height = 0.4;
simple_robot.joints(2).props = {};

simple_robot.frame.scale = 0.4;
simple_robot.frame.width = 0.05;

gripper.aperture = 0.5;
gripper.stroke = 0.4;
gripper.height = 0.25;
gripper.R0 = rot(y0,pi/2)*rot(z0,pi/2);

%%%%%% Create Environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
% create robot
h_robot = createRobot(eye(3), zed, simple_robot,'CreateFrames','on');
% Create gripper
h_gripper = createParallelJawGripper(rot(y0,pi/2)*rot(z0,pi/2), zed, ...
                                                        gripper);
% move gripper to the end effector of the robot
h_gripper = updateRigidBody(eye(3), sum(simple_robot.P,2), h_gripper);
% attach gripper and combine into single robot handle
h_robot = combineRobots(h_robot,h_gripper);
axis equal;
axis([-3.5 3.5 -3.5 3.5 -1 1]);
view([0 90])

% place object in world
load_param.radius = 0.2;
load = createEllipsoid(eye(3),[0;0;0],load_param,'FaceColor',[0;1;1]);
load = updateRigidBody(eye(3), [0;3.25;0], load);
load.labels = attachPrefix('load_', load.labels);


%% Animate robot with joint displacements

dt = 0.01;

% pre-defined robot motion
t = 0:dt:1;
q1 = pi/2*t;
q2 = 0*t;

% Move robot end effector towards object
for k=1:length(t)
    tic;
    q = {{[q1(k);q2(k)];0;0}};
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end

pause(dt);

% grab object
q1 = q1(end); q2 = q2(end);
if norm(h_robot.robots(1).frames(end).t - load.t) <= 0.5
    % Animate gripper closing
    t = 0:dt:0.2;
    qg = (gripper.aperture - 2*load_param.radius)*t/t(end);
    for k=1:length(t)
        tic;
        q = {{[q1;q2];qg(k)/2;qg(k)/2}};
        h_robot = updateRobot(q,h_robot);
        drawnow;
        t1 = toc;
        while t1 < dt,   t1 = toc;   end
    end
    h_robot = graspLoad(load,h_robot);
else
    disp('Could not locate object: ')
    disp(['dist: ' num2str(norm(h_robot.frame(end).t - load.t))]);
end

pause(dt);

% pre-defined robot motion
t = 0:dt:2;
q1 = q1 - pi/2*t;
q2 = q2 - 3*pi/8*t;
qg = gripper.aperture - 2*load_param.radius;

% move robot to desired new location
for k=1:length(t)
    tic;
    q = {{[q1(k);q2(k)];qg/2;qg/2}};
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end

pause(dt);

% release object
q1 = q1(end); q2 = q2(end);
if (~isempty(h_robot.robots(1).load))
    % Animate gripper opening
    t = 0:dt:0.2;
    qg = (gripper.aperture - 2*load_param.radius)*(1 - t/t(end));
    for k=1:length(t)
        tic;
        q = {{[q1;q2];qg(k)/2;qg(k)/2}};
        h_robot = updateRobot(q,h_robot);
        drawnow;
        t1 = toc;
        while t1 < dt,   t1 = toc;   end
    end
    [h_robot, load] = releaseLoad(h_robot);
end

pause(dt);

% return robot to zero position
t = 0:dt:1;
q1 = q1 - q1*t/t(end);
q2 = q2 - q2*t/t(end);

for k=1:length(t)
    tic;
    q = {{[q1(k);q2(k)];0;0}};
    h_robot = updateRobot(q,h_robot);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
