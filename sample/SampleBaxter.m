% Creation of a pre-defined robot
% This is a test script to demonstrate how to take advantage of the
% pre-defined robots within the library.  In this example, we take a
% Rethink Robotics 'Baxter' robot and animate it following a prescribed
% path
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

figure(1);
baxter = createBaxter(eye(3), [0;0;0], 'CreateFrames','off');
axis equal;
axis([-2 2 -2 2 -.85 1]);
view([75 10]);

%% Simple Animation

t = 0:0.02:2;

ql = zeros(7,1); % left arm joint angles
qr = zeros(7,1); % right arm joint angles
qh = 0; % head pan angle

for k=1:length(t);    
    ql(2) = pi/4*sin(2*pi*t(k)); % s1 in left arm
    qr(2) = -pi/4*sin(2*pi*t(k)); % s1 in right arm
    qh(1) = sin(2*pi*t(k)); % head pan
    
    q = {ql;qr;qh};
    
    baxter = updateRobot(q,baxter);
    drawnow;
end
