clear all; close all;

% simple animation of body


cyl_param.radius = 0.1;
cyl_param.height = 0.3;


figure(1);
cyl = createCylinder(rot([0;1;0],pi/2),[0;0.3;0.1],cyl_param,'FaceColor',[0;0;1]);
axis equal; axis([-1 1 -1 1 0 1]); grid on;

%% Animate rolling across floor

dt = 0.001;
t = 0:dt:1;

w = 2*pi;

tic;
for k=1:length(t)
    R = rot([1;0;0],w*t(k));
    p = cyl.t + hat([w;0;0])*[0;0;0.1]*dt;
    figure(1);
    cyl = updateRigidBody(R,p,cyl);
    drawnow;
end
toc;