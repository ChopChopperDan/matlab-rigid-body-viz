% Simple animation of a rigid body.  
% This script will create a Cylinder object with user-defined parameters
% laying on its side, and then rolling along a surface with a constant
% angular velocity.  It is intended to demonstrate a simple creation of a
% body and how to update the body to desired poses.

clear all; close all;

% Add path to drawing library if not already included
if isempty(strfind(path, 'matlab-rigid-body-viz'))
    addpath('./..');
    rigidbodyviz_setup();
    rmpath('./..');
end

% Define body
cyl_param.radius = 0.1;
cyl_param.height = 0.3;
cyl_props = {'FaceColor',[0;1;1],'FaceAlpha',0.5};

% Define 3d-frame
frame_param.scale = 0.15;
frame_param.width = 0.02;

R0 = rot([0;1;0],pi/2);
p0 = [0;0.3;0.1];

figure(1);
cyl = createCylinder(R0,p0,cyl_param,cyl_props{:});
% Attach 3D Frame to cylinder
frame = create3DFrame(R0,p0,frame_param);
cyl.bodies = [cyl.bodies frame.bodies];
cyl.labels = [cyl.labels frame.labels];
axis equal; axis([-1 1 -1 1 0 1]); grid on;

%% Animate rolling across floor

% time stepping of animation
dt = 0.02;
t = 0:dt:2;

% angular velocity of cylinder
w = 2*pi;

for k=1:length(t)
    % Calculate new rotation and position at time t(k)
    R = rot([1;0;0],w*t(k));
    p = cyl.t + hat([w;0;0])*[0;0;0.1]*dt;
    figure(1);
    cyl = updateRigidBody(R,p,cyl);
    drawnow;
end
