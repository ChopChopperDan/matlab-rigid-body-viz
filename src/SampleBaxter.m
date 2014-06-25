clear all; close all;

figure(1);
baxter = createBaxter('CreateFrames','off');
axis equal;
axis([-2 2 -2 2 -.85 1]);

%% Simple Animation

t = 0:0.01:2;

ql = zeros(7,1);
qr = zeros(7,1);
qh = 0;

for k=1:length(t);    
    ql(2) = pi/4*sin(2*pi*t(k));
    qr(2) = -pi/4*sin(2*pi*t(k));
    qh(1) = sin(2*pi*t(k));
    
    baxter.left_arm = updateRobot(ql,baxter.left_arm);
    baxter.right_arm = updateRobot(qr,baxter.right_arm);
    baxter.head = updateRobot(qh,baxter.head);
    drawnow;
end