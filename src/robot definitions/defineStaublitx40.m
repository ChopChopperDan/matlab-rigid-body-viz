function tx40_const = defineStaublitx40(varargin)
    %
    % tx40_const = defineStaublitx40()
    % tx40_const = defineStaublitx40(...) - allows additional optional
    %                                    parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % define-file for the Staubli tx40 Robot.  Returns struct with the
    % following form:
    %
    % root
    %   -> name         : string denoting the name of the robot
    %   -> kin          : container for kinematics
    %       -> H                 : [3 x 6] joint axes
    %       -> P                 : [3 x 7] inter-joint displacements
    %       -> joint_type        : [1 x 6] joint types
    %   -> limit        : container for kinematic and dynamic joint limits
    %       -> upper_joint_limit : [1 x 6] [rad]
    %       -> lower_joint_limit : [1 x 6] [rad]
    %       -> velocity_limit    : [1 x 6] [rad/s]
    %   -> vis          : container for visualization
    %       -> joints            :  6 index struct array containing fields
    %                               param
    %                               props
    %       -> links             :  7 index cell array containing fields
    %                               handle
    %                               R
    %                               t
    %                               param
    %                               props
    %       -> frame             : struct containing appropriate sized
    %                               dimensions for visualizing the 
    %                               3D frames
    %
    % see also CREATEROBOT
    
    x0 = [1;0;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    flags = {'Origin'};
    defaults = {[eye(3) zed; zed' 1]};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    
    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    tx40_const = defineEmptyRobot();
    
    % Name
    tx40_const.name = 'tx40';
    
    % Kinematic constants
    tx40_const.kin.H = R0*[z0 x0 x0 z0 x0 z0];
    tx40_const.kin.P = R0*[.32*z0 zed [.035;0;.225] zed .225*z0 .065*z0 zed];
    tx40_const.kin.P(:,1) = t0 + tx40_const.kin.P(:,1);
    tx40_const.kin.joint_type = zeros(1,6);
    
    % Joint limits
    tx40_const.limit.upper_joint_limit = [180 125 138 270 133.5 270]*pi/180;
    tx40_const.limit.lower_joint_limit = [-180 -125 -138 -270 -120 -270]*pi/180;
    tx40_const.limit.velocity_limit = [555 475 585 1035 1135 1575]*pi/180;
    
    % Visualization constants
    link_props = {'FaceColor', [.9;0.9;0.9], 'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0.9;0.9;0.9]};
    joint_radius = [0.084 0.082 0.061 0.0585 0.04 0.02];
    joint_height = [0.165 0.1855 0.117 0.124 0.093 0.005];
    
    % Define joints
    tx40_const.vis.joints = struct('param',cell(1,6),'props',cell(1,6));
    for n=1:6
        tx40_const.vis.joints(n).param = struct('radius',joint_radius(n), ...
                                            'height',joint_height(n));
        tx40_const.vis.joints(n).props = joint_props;
    end
    
    % Define links
    tx40_const.vis.links = struct('handle',cell(1,7), ...
                            'R',cell(1,7),'t',cell(1,7), ...
                            'param',cell(1,7),'props',cell(1,7));
    tx40_const.vis.links(1).handle = @createCylinder;
    tx40_const.vis.links(1).R = eye(3);
    tx40_const.vis.links(1).t = 0.115*z0;
    tx40_const.vis.links(1).param = struct('radius',0.084,'height',0.230);
    tx40_const.vis.links(1).props = link_props;
    
    tx40_const.vis.links(3).handle = @createCylinder;
    tx40_const.vis.links(3).R = eye(3);
    tx40_const.vis.links(3).t = [.1655;0;.1125];
    tx40_const.vis.links(3).param = struct('radius',0.072,'height',0.368);
    tx40_const.vis.links(3).props = link_props;
    
    tx40_const.vis.links(5).handle = @createCylinder;
    tx40_const.vis.links(5).R = eye(3);
    tx40_const.vis.links(5).t = .1235*z0;
    tx40_const.vis.links(5).param = struct('radius',0.0585,'height',0.123);
    tx40_const.vis.links(5).props = link_props;
    
    tx40_const.vis.links(6).handle = @createCylinder;
    tx40_const.vis.links(6).R = eye(3);
    tx40_const.vis.links(6).t = 0.0513*z0;
    tx40_const.vis.links(6).param = struct('radius',0.02,'height',0.0225);
    tx40_const.vis.links(6).props = link_props;
     
    tx40_const.vis.frame = struct('scale', 0.1, 'width', 0.01);
    
end