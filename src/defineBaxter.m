function baxter_const = defineBaxter(origin)
    %
    % baxter_const = defineBaxter()
    % baxter_const = defineBaxter(origin) - origin is [4 x 4] matrix
    % denoting orientation and translation of Baxter with respect to the
    % world frame
    %
    % define-file for the Rethink Robotics Baxter.  Returns struct with the
    % following form:
    %
    % root
    %   -> left_arm
    %       -> H        : [3 x 7] joint axes
    %       -> P        : [3 x 8] rigid translation between each joint
    %       -> type     : [1 x 7] joint types
    %       -> n        : scalar number of joints
    %       -> origin   : [4 x 4] transformation matrix to origin
    %
    %       -> upper_joint_limit :  [7 x 1] upper joint limits [rad]
    %       -> lower_joint_limit :  [7 x 1] lower joint limits [rad]
    %       -> velocity_limit    :  [7 x 1] velocity limits    [rad/s]
    %       -> effort_limit      :  [7 x 1] effort limits      [Nm/s]
    %
    %       -> joint_radius      :  [7 x 1] radii of each joint [m]
    %       -> joint_height      :  [7 x 1] height of each joint [m]
    %   -> right_arm
    %       -> H        : [3 x 7] joint axes
    %       -> P        : [3 x 8] rigid translation between each joint
    %       -> type     : [1 x 7] joint types
    %       -> n        : scalar number of joints
    %       -> origin   : [4 x 4] transformation matrix to origin
    %
    %       -> upper_joint_limit :  [7 x 1] upper joint limits [rad]
    %       -> lower_joint_limit :  [7 x 1] lower joint limits [rad]
    %       -> velocity_limit    :  [7 x 1] velocity limits    [rad/s]
    %       -> effort_limit      :  [7 x 1] effort limits      [Nm/s]
    %
    %       -> joint_radius      :  [7 x 1] radii of each joint [m]
    %       -> joint_height      :  [7 x 1] height of each joint [m]
    %   -> head
    %       -> H        : [3 x 1] joint axes
    %       -> P        : [3 x 2] rigid translation between each joint
    %       -> type     : [1 x 1] joint types
    %       -> n        : scalar number of joints
    %       -> origin   : [4 x 4] transformation matrix to origin
    %
    %       -> upper_joint_limit :  [7 x 1] upper joint limits [rad]
    %       -> lower_joint_limit :  [7 x 1] lower joint limits [rad]
    %
    %   see also CREATEBAXTER 
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end

    % Kinematic Constants
    left_arm.H = origin(1:3,1:3)*rot(z0,pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    left_arm.P = origin(1:3,1:3)*[origin(1:3,4)+[0.06375;.25888;0.119217], ...
                    rot(z0,pi/4)*[[0.069;0;0.27035], ...
                    zed, [0.36435;0;-0.069], ...
                    zed, [0.37429;0;-0.01], ...
                    zed, 0.229525*x0]];
    left_arm.type = zeros(1,7);
    left_arm.n = 7;
    left_arm.origin = [eye(3) zed; zed' 1];
    
    right_arm.H = origin(1:3,1:3)*rot(z0,-pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    right_arm.P = origin(1:3,1:3)*[origin(1:3,4)+[0.06375;-.25888;0.119217], ...
                    rot(z0,-pi/4)*[[0.069;0;0.27035], ...
                    zed, [0.36435;0;-0.069], ...
                    zed, [0.37429;0;-0.01], ...
                    zed, 0.229525*x0]];
    right_arm.type = zeros(1,7);
    right_arm.n = 7;
    right_arm.origin = [eye(3) zed; zed' 1];
    
    head.H = origin(1:3,1:3)*z0;
    head.P = origin(1:3,1:3)*[origin(1:3,4) + [.0599;0;.6955] zed];
    head.type = 0;
    head.n = 1;
    head.origin = [eye(3) zed; zed' 1];
    
    % Dynamic Constants
    left_arm.upper_joint_limit = [97.5; 60; 175; 150; 175; 120; 175]*pi/180;
    left_arm.lower_joint_limit = [-97.5;-123; -175; -2.5; -175; -90; -175]*pi/180;
    left_arm.velocity_limit = [2.5;2.5;2.5;2.5;5;5;5];
    left_arm.torque_limit = [60;60;60;60;20;20;20];
    
    right_arm.upper_joint_limit = [97.5; 60; 175; 150; 175; 120; 175]*pi/180;
    right_arm.lower_joint_limit = [-97.5;-123; -175; -2.5; -175; -90; -175]*pi/180;
    right_arm.velocity_limit = [2.5;2.5;2.5;2.5;5;5;5];
    right_arm.torque_limit = [60;60;60;60;20;20;20];
    
    head.upper_joint_limit = pi/2;
    head.lower_joint_limit = -pi/2;
    
    % Visualization constants
    left_arm.joint_radius = [.075;.075;.075;.07;.07;.06;.06];
    left_arm.joint_height = [.1;.19;.15;.18;.14;.15;.12];
    
    right_arm.joint_radius = [.075;.075;.075;.07;.07;.06;.06];
    right_arm.joint_height = [.1;.19;.15;.18;.14;.15;.12];
    
    
    baxter_const.left_arm = left_arm;
    baxter_const.right_arm = right_arm;
    baxter_const.head = head;
end