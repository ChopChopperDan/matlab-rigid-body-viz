function tx40_const = defineStaublitx40(origin)
    %
    % tx40_const = defineStaublitx40()
    % tx40_const = defineStaublitx40(origin) - origin is [4 x 4] matrix
    % denoting orientation and translation of Staubli tx40 with respect 
    % to the world frame
    %
    % define-file for the Staubli tx40 Robot.  Returns struct with the
    % following form:
    %
    % root
    %   -> H        : [3 x 6] joint axes
    %   -> P        : [3 x 7] rigid translation between each joint
    %   -> type     : [1 x 6] joint types
    %   -> n        : scalar number of joints
    %   -> origin   : [4 x 4] transformation matrix to origin
    %
    %   -> joint_radius      :  [7 x 1] radii of each joint [m]
    %   -> joint_height      :  [7 x 1] height of each joint [m]
    %
    % see also CREATESTAUBLITX40
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    % Kinematic constants
    tx40_const.H = origin(1:3,1:3)*[z0 x0 x0 z0 x0 z0];
    tx40_const.P = origin(1:3,1:3)*[.32*z0 zed ...
                                [.035;0;.225] zed .225*z0 .065*z0 zed];
    tx40_const.P(:,1) = origin(1:3,4) + tx40_const.P(:,1);
    tx40_const.type = zeros(1,6);
    tx40_const.n = 6;
    tx40_const.origin = [eye(3) zed; zed' 1];
    
    % Visualization constants
    tx40_const.joint_radius = [0.084 0.082 0.061 0.0585 0.04 0.02];
    tx40_const.joint_height = [0.165 0.1855 0.117 0.124 0.093 0.005];
    
end