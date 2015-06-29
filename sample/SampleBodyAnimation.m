% Simple animation of a rigid body.  
% This script will create a Cylinder object with user-defined parameters
% laying on its side, and then rolling along a surface with a constant
% angular velocity.  It is intended to demonstrate a simple creation of a
% body and how to update the body to desired poses.
%
% Be sure to use rigidbodyviz_setup() before running

clear variables; close all;

% Define body
cyl_param = struct('radius', 0.1, 'height', 0.3);
cyl_props = {'FaceColor',[0;1;1],'FaceAlpha',0.5};

% Define 3d-frame
frame_param = struct('scale', 0.15, 'width', 0.02);

% Cylinder is defined with its long axis along z.  This rotation will place
% that axis along the x axis
R0 = rot([0;1;0],pi/2);
% Desired initial position for the center of the cylinder
p0 = [0;0.3;0.1];

figure(1);
cyl = createCylinder(R0,[0;0;0],cyl_param,cyl_props{:});
% Attach 3D Frame to cylinder
cyl = combineRigidBodies(cyl, create3DFrame(R0,[0;0;0],frame_param));
% Place in initial position
cyl = updateRigidBody(eye(3),p0,cyl);
axis equal; axis([-1 1 -1 1 0 1]); grid on;

%% Animate rolling across floor

% time sequence
T = 4; dt = 0.02;
t = 0:dt:T;

% angular velocity of cylinder
w = 3*pi/T;

for k=1:length(t)
    % Calculate new rotation and position at time t(k)
    tic;
    R = rot([1;0;0],w*t(k));
    p = cyl.t + hat([w;0;0])*[0;0;0.1]*dt;
    figure(1);
    cyl = updateRigidBody(R,p,cyl);
    drawnow;
    t1 = toc;
    while t1 < dt,   t1 = toc;   end
end
