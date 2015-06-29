% Creation of a pre-defined robot
% This is a test script to demonstrate how to take advantage of the
% pre-defined robots within the library.  In this example, we take a
% Rethink Robotics 'Baxter' robot and animate it following a prescribed
% path
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

[baxter_const, baxter_structure] = defineBaxter();

figure(1);
baxter = createCombinedRobot(baxter_const, baxter_structure);
axis equal;
axis([-2 2 -2 2 -.85 1]);
view([75 10]);

%% Simple Animation

% Time sequence
T = 2; dt = 0.02;
t = 0:dt:T;

% Pre-allocate angle structure
q.names = {baxter_const.name};
q.states = {zeros(7,1) zeros(7,1) 0};

ql = zeros(7,1); % left arm joint angles
qr = zeros(7,1); % right arm joint angles
qh = 0; % head pan angle

for k=1:length(t);
    tic;
    ql(4) = pi/4*sin(2*pi*t(k)); % s1 in left arm
    qr(4) = -pi/4*sin(2*pi*t(k)); % s1 in right arm
    qh(1) = sin(2*pi*t(k)); % head pan
    
    q.states = {ql qr qh};
    baxter = updateRobot(q, baxter);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc; end
end
