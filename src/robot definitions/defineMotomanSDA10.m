function sda10_const = defineMotomanSDA10(origin)
    %
    % sda10_const = defineMotomanSDA10()
    % sda10_const = defineMotomanSDA10(origin) - origin is [4 x 4] matrix
    % denoting orientation and translation of Motoman SDA10 with respect 
    % to the world frame
    %
    % define-file for the Motoman SDA10 Robot.  Returns struct with the
    % following form:
    %
    % root
    %   -> left_arm
    %       -> H        : [3 x 8] joint axes
    %       -> P        : [3 x 9] rigid translation between each joint
    %       -> type     : [1 x 8] joint types
    %       -> n        : scalar number of joints
    %       -> origin   : [4 x 4] transformation matrix to origin
    %
    %       -> upper_joint_limit :  [8 x 1] upper joint limits [rad]
    %       -> lower_joint_limit :  [8 x 1] lower joint limits [rad]
    %
    %       -> joint_radius      :  [8 x 1] radii of each joint [m]
    %       -> joint_height      :  [8 x 1] height of each joint [m]
    %   -> right_arm
    %       -> H        : [3 x 8] joint axes
    %       -> P        : [3 x 9] rigid translation between each joint
    %       -> type     : [1 x 8] joint types
    %       -> n        : scalar number of joints
    %       -> origin   : [4 x 4] transformation matrix to origin
    %
    %       -> upper_joint_limit :  [8 x 1] upper joint limits [rad]
    %       -> lower_joint_limit :  [8 x 1] lower joint limits [rad]
    %
    %       -> joint_radius      :  [8 x 1] radii of each joint [m]
    %       -> joint_height      :  [8 x 1] height of each joint [m]
    %
    %   see also CREATEMOTOMANSDA10
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end

    left_arm.H = origin(1:3,1:3)*[z0 x0 y0 x0 y0 z0 y0 z0];
    left_arm.P = origin(1:3,1:3)*[0.9*z0 [0.05;-.1;.35] .215*x0 ...
                        zed .36*x0 .36*z0 zed .155*z0 zed];
    left_arm.P(:,1) = origin(1:3,4) + left_arm.P(:,1);
    left_arm.type = zeros(8,1);
    left_arm.n = 8;
    left_arm.origin = [eye(3) zed; zed' 1];
    
    right_arm.H = origin(1:3,1:3)*[z0 x0 y0 x0 y0 z0 y0 z0];
    right_arm.P = origin(1:3,1:3)*[0.9*z0 [0-.05;-.1;.35] -.215*x0 ...
                        zed -.36*x0 .36*z0 zed .155*z0 zed];
    right_arm.P(:,1) = origin(1:3,4) + right_arm.P(:,1);
    right_arm.type = zeros(8,1);
    right_arm.n = 8;
    right_arm.origin = [eye(3) zed; zed' 1];
    
    left_arm.lower_joint_limit = [-170; -90;- 110; -170; -45; ...
                                        -180; -110; -180]*pi/180;
    left_arm.upper_joint_limit = [170; 270; 110; 170; 225; ...
                                        180; 110; 180]*pi/180;
    
    right_arm.upper_joint_limit = [170; 270; 110; 170; 45; ...
                                        180; 110; 180]*pi/180;
    right_arm.lower_joint_limit = [-170; -90; -110; -170; -225; ...
                                        -180; -110; -180]*pi/180;
    
    left_arm.joint_radius = [0.12;0.08;0.08;0.08;0.06;0.05;0.05;0.045];
    left_arm.joint_height = [0.1;0.1;0.16;0.16;0.12;0.1;0.1;0.02];
    right_arm.joint_radius = left_arm.joint_radius;
    right_arm.joint_height = left_arm.joint_height;
    
    sda10_const.left_arm = left_arm;
    sda10_const.right_arm = right_arm;
end