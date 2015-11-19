function [sda10d_const, sda10d_structure] = defineMotomanSDA10D(varargin)
    %
    % [sda10d_const, sda10d_structure] = defineMotomanSDA10D()
    % [sda10d_const, sda10d_structure] = defineMotomanSDA10D(...)
    %                           allows additional optional parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % define-file for the Motoman SDA10D Robot.  Returns struct array with 
    %   the following fields:
    %
    % root
    %   -> name             : (1) 'sda10d_torso'
    %                         (2) 'sda10d_right_arm'
    %                         (3) 'sda10d_left_arm'
    %   -> kin
    %       -> H            : joint axes
    %       -> P            : inter-joint translation
    %       -> joint_type   : joint types
    %   -> limit
    %       -> upper_joint_limit :  upper joint limits [rad]
    %       -> lower_joint_limit :  lower joint limits [rad]
    %       -> velocity_limit:   :  velocity limits [rad/s]
    %   -> vis
    %       -> joints       :  cell array each joint [m]
    %       -> links        :  height of each joint [m]
    %       -> frame        :  frame dimensions
    %       -> peripherals  :  empty
    %
    %   see also CREATECOMBINEDROBOT
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    flags = {'Origin'};
    defaults = {[eye(3) zed; zed' 1]};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    
    R0 = origin(1:3,1:3); t0 = origin(1:3,4);
    
    link_props = {'FaceColor', [0.8;0.8;0.8], 'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0;0;1]};
    
    % Properties of both arms
    joint_radius = [0.08;0.08;0.08;0.06;0.05;0.05;0.045];
    joint_height = [0.1;0.16;0.16;0.12;0.1;0.1;0.02];    
    lower_joint_limit = [-90 -110 -170 -45 -180 -110 -180]*pi/180;
    upper_joint_limit = [270 110 170 225 180 110 180]*pi/180;
    velocity_limit = [170 170 170 170 200 200 400]*pi/180;
    
    % Grab standard robot structure
    sda10d_const = defineEmptyRobot(3);
    
    %%% Torso joint    
    sda10d_const(1).name = 'sda10d_torso';
    
    % Kinematics
    sda10d_const(1).kin.H = R0*z0;
    sda10d_const(1).kin.P = R0*[0.9*z0 zed];
    sda10d_const(1).kin.P(:,1) = t0 + sda10d_const(1).kin.P(:,1);
    sda10d_const(1).kin.joint_type = 0;
    
    % Limits
    sda10d_const(1).limit.lower_joint_limit = -170*pi/180;
    sda10d_const(1).limit.upper_joint_limit = 170*pi/180;
    sda10d_const(1).limit.velocity_limit = 133.33*pi/180;
    
    % Visualization parameters
    sda10d_const(1).vis.joints.param = struct('radius', 0.12,'height',0.1);
    sda10d_const(1).vis.joints.props = joint_props;
    
    sda10d_const(1).vis.links = struct('handle',cell(1,2), ...
                            'R',cell(1,2),'t',cell(1,2), ...
                            'param',cell(1,2),'props',cell(1,2));
    
    sda10d_const(1).vis.links(1).handle = @createCylinder;
    sda10d_const(1).vis.links(1).R = eye(3);
    sda10d_const(1).vis.links(1).t = [0;0;0.425];
    sda10d_const(1).vis.links(1).param = struct('radius',0.12,'height',0.85);
    sda10d_const(1).vis.links(1).props = link_props;
    
    sda10d_const(1).vis.links(2).handle = @createCylinder;
    sda10d_const(1).vis.links(2).R = rot(x0,18*pi/180);
    sda10d_const(1).vis.links(2).t = rot(x0,18*pi/180)*0.145*z0;
    sda10d_const(1).vis.links(2).param = struct('radius',0.12,'height',0.19);
    sda10d_const(1).vis.links(2).props = link_props;
    
    sda10d_const(1).vis.frame = struct('scale',0.15,'width',0.01);
    
    
    %%% Right arm
    sda10d_const(2).name = 'sda10d_right_arm';
    
    % Kinematics
    sda10d_const(2).kin.H = R0*[x0 y0 x0 y0 z0 y0 z0];
    sda10d_const(2).kin.P = R0*[[-0.05;-.1;.3] -.215*x0 ...
                        zed -.36*x0 .36*z0 zed .145*z0 0.01*z0];
    sda10d_const(2).kin.P(:,1) = t0 + sda10d_const(2).kin.P(:,1);
    sda10d_const(2).kin.joint_type = zeros(1,7);
    
    % Limits
    sda10d_const(2).limit.upper_joint_limit = upper_joint_limit;
    sda10d_const(2).limit.lower_joint_limit = lower_joint_limit;
    sda10d_const(2).limit.velocity_limit = velocity_limit;
    
    % Visualization parameters
    sda10d_const(2).vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        sda10d_const(2).vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        sda10d_const(2).vis.joints(n).props = joint_props;
    end
    
    sda10d_const(2).vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    sda10d_const(2).vis.links(2).handle = @createCylinder;
    sda10d_const(2).vis.links(2).R = rot(y0,pi/2);
    sda10d_const(2).vis.links(2).t = -.0925*x0;
    sda10d_const(2).vis.links(2).param = struct('radius',0.08, ...
                                                'height',0.085);
    sda10d_const(2).vis.links(2).props = link_props;
    
    sda10d_const(2).vis.links(4).handle = @createCylinder;
    sda10d_const(2).vis.links(4).R = rot(y0,pi/2);
    sda10d_const(2).vis.links(4).t = -.19*x0;
    sda10d_const(2).vis.links(4).param = struct('radius',0.06, ...
                                                'height',0.22);
    sda10d_const(2).vis.links(4).props = link_props;
    
    sda10d_const(2).vis.links(5).handle = @createCylinder;
    sda10d_const(2).vis.links(5).R = eye(3);
    sda10d_const(2).vis.links(5).t = .185*z0;
    sda10d_const(2).vis.links(5).param = struct('radius',0.05, ...
                                                'height',0.25);
    sda10d_const(2).vis.links(5).props = link_props;
    
    sda10d_const(2).vis.links(7).handle = @createCylinder;
    sda10d_const(2).vis.links(7).R = eye(3);
    sda10d_const(2).vis.links(7).t = .0925*z0;
    sda10d_const(2).vis.links(7).param = struct('radius',0.045, ...
                                                'height',0.085);
    sda10d_const(2).vis.links(7).props = link_props;
    
    sda10d_const(2).vis.frame = struct('scale',0.15,'width',0.01);
     
    %%% Left Arm
    sda10d_const(3).name = 'sda10d_left_arm';
    % Kinematics
    sda10d_const(3).kin.H = R0*[x0 y0 x0 y0 z0 y0 z0];
    sda10d_const(3).kin.P = R0*[[0.05;-.1;.3] .215*x0 ...
                        zed .36*x0 .36*z0 zed .145*z0 0.01*z0];
    sda10d_const(3).kin.P(:,1) = t0 + sda10d_const(3).kin.P(:,1);
    sda10d_const(3).kin.joint_type = zeros(1,7);
    
    % Limits
    sda10d_const(3).limit.upper_joint_limit = upper_joint_limit;
    sda10d_const(3).limit.lower_joint_limit = lower_joint_limit;
    sda10d_const(3).limit.velocity_limit = velocity_limit;
    
    % Visualization parameters
    sda10d_const(3).vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        sda10d_const(3).vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        sda10d_const(3).vis.joints(n).props = joint_props;
    end
    
    sda10d_const(3).vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    sda10d_const(3).vis.links(2).handle = @createCylinder;
    sda10d_const(3).vis.links(2).R = rot(y0,pi/2);
    sda10d_const(3).vis.links(2).t = .0925*x0;
    sda10d_const(3).vis.links(2).param = struct('radius',0.08, ...
                                                'height',0.085);
    sda10d_const(3).vis.links(2).props = link_props;
    
    sda10d_const(3).vis.links(4).handle = @createCylinder;
    sda10d_const(3).vis.links(4).R = rot(y0,pi/2);
    sda10d_const(3).vis.links(4).t = .19*x0;
    sda10d_const(3).vis.links(4).param = struct('radius',0.06, ...
                                                'height',0.22);
    sda10d_const(3).vis.links(4).props = link_props;
    
    sda10d_const(3).vis.links(5).handle = @createCylinder;
    sda10d_const(3).vis.links(5).R = eye(3);
    sda10d_const(3).vis.links(5).t = .185*z0;
    sda10d_const(3).vis.links(5).param = struct('radius',0.05, ...
                                                'height',0.25);
    sda10d_const(3).vis.links(5).props = link_props;
    
    sda10d_const(3).vis.links(7).handle = @createCylinder;
    sda10d_const(3).vis.links(7).R = eye(3);
    sda10d_const(3).vis.links(7).t = .0925*z0;
    sda10d_const(3).vis.links(7).param = struct('radius',0.045, ...
                                                'height',0.085);
    sda10d_const(3).vis.links(7).props = link_props;
    
    sda10d_const(3).vis.frame = struct('scale',0.15,'width',0.01);
    
    
    %%% Build structure with both arms attached to the end of the torso
    sda10d_structure = defineEmptyRobotStructure(3);
    [sda10d_structure.name] = sda10d_const.name;
    [sda10d_structure(2:3).left] = deal(sda10d_const(1).name);
    sda10d_structure(1).right = {sda10d_const(2:3).name};
end