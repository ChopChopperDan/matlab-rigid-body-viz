% Create of a pre-defined robot
% This is a test script to demonstrate how to take advantage of the
% pre-defined robots within the library.  In this example, we take a
% Rethink Robotics 'Baxter' robot and animate it following a prescribed
% path

clear all; close all;


% Add path to drawing library if not already included
if isempty(strfind(path, 'matlab-rigid-body-viz'))
    addpath('./..');
    rigidbodyviz_setup();
    rmpath('./..');
end

figure(1);
baxter = createBaxter('CreateFrames','off');
axis equal;
axis([-2 2 -2 2 -.85 1]);

%% Simple Animation

t = 0:0.02:2;

ql = zeros(7,1); % left arm joint angles
qr = zeros(7,1); % right arm joint angles
qh = 0; % head pan angle

for k=1:length(t);    
    ql(2) = pi/4*sin(2*pi*t(k)); % s1 in left arm
    qr(2) = -pi/4*sin(2*pi*t(k)); % s1 in right arm
    qh(1) = sin(2*pi*t(k)); % head pan
    
    baxter.left_arm = updateRobot(ql,baxter.left_arm);
    baxter.right_arm = updateRobot(qr,baxter.right_arm);
    baxter.head = updateRobot(qh,baxter.head);
    drawnow;
end
