function sda10_const = defineMotomanSDA10D(origin)
    %
    % sda10_const = defineMotomanSDA10D()
    % sda10_const = defineMotomanSDA10D(origin) - origin is [4 x 4] matrix
    % denoting orientation and translation of Motoman SDA10 with respect 
    % to the world frame
    %
    % define-file for the Motoman SDA10D Robot.  Returns struct with the
    % following form:
    %
    % root
    %   -> torso_const
    %       -> name             : 'sda10d_torso'
    %       -> kin
    %           -> H            : [3 x 1] joint axes
    %           -> P            : [3 x 2] inter-joint translation
    %           -> joint_type   : [1 x 1] joint types
    %       -> limit
    %           -> upper_joint_limit :  [1 x 1] upper joint limits [rad]
    %           -> lower_joint_limit :  [1 x 1] lower joint limits [rad]
    %           -> velocity_limit:   :  [1 x 1] velocity limits [rad/s]
    %       -> vis
    %           -> joints       :  [1 x 1] cell array each joint [m]
    %           -> links        :  [1 x 1] height of each joint [m]
    %           -> frame        :  [1 x 1] frame dimensions
    %   -> left_arm_const
    %       -> name             : 'sda10d_left_arm'
    %       -> kin
    %           -> H            : [3 x 7] joint axes
    %           -> P            : [3 x 8] inter-joint translation
    %           -> joint_type   : [1 x 7] joint types
    %       -> limit
    %           -> upper_joint_limit :  [1 x 7] upper joint limits [rad]
    %           -> lower_joint_limit :  [1 x 7] lower joint limits [rad]
    %           -> velocity_limit:   :  [1 x 7] velocity limits [rad/s]
    %
    %       -> vis
    %           -> joints       :  [1 x 7] cell array each joint [m]
    %           -> links        :  [1 x 8] height of each joint [m]
    %           -> frame        :  frame dimensions
    %   -> right_arm_const
    %       -> name             : 'sda10d_right_arm'
    %       -> kin
    %           -> H            : [3 x 7] joint axes
    %           -> P            : [3 x 8] inter-joint translation
    %           -> joint_type   : [1 x 7] joint types
    %       -> limit
    %           -> upper_joint_limit :  [1 x 7] upper joint limits [rad]
    %           -> lower_joint_limit :  [1 x 7] lower joint limits [rad]
    %           -> velocity_limit:   :  [1 x 7] velocity limits [rad/s]
    %
    %       -> vis
    %           -> joints       :  [1 x 7] cell array each joint [m]
    %           -> links        :  [1 x 8] height of each joint [m]
    %           -> frame        :  frame dimensions
    %
    %   see also CREATEMOTOMANSDA10D
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end
    
    R0 = origin(1:3,1:3); t0 = origin(1:3,4);
    
    link_props = {'FaceColor', [0.8;0.8;0.8], 'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0;0;1]};
    
    % Properties of both arms
    joint_radius = [0.08;0.08;0.08;0.06;0.05;0.05;0.045];
    joint_height = [0.1;0.16;0.16;0.12;0.1;0.1;0.02];    
    lower_joint_limit = [-90 -110 -170 -45 -180 -110 -180]*pi/180;
    upper_joint_limit = [270 110 170 225 180 110 180]*pi/180;
    velocity_limit = [170 170 170 170 200 200 400]*pi/180;
    
    %%% Torso joint
    torso_const.name = 'sda10d_torso';
    % Kinematics
    torso_const.kin.H = R0*z0;
    torso_const.kin.P = R0*[0.9*z0 zed];
    torso_const.kin.P(:,1) = t0 + torso_const.kin.P(:,1);
    torso_const.kin.joint_type = 0;
    
    % Limits
    torso_const.limit.lower_joint_limit = -170*pi/180;
    torso_const.limit.upper_joint_limit = 170*pi/180;
    torso_const.limit.velocity_limit = 133.33*pi/180;
    
    % Visualization parameters
    torso_const.vis.joints.param = struct('radius', 0.12,'height',0.1);
    torso_const.vis.joints.props = joint_props;
    
    torso_const.vis.links = struct('handle',cell(1,2), ...
                            'R',cell(1,2),'t',cell(1,2), ...
                            'param',cell(1,2),'props',cell(1,2));
    
    torso_const.vis.links(1).handle = @createCylinder;
    torso_const.vis.links(1).R = eye(3);
    torso_const.vis.links(1).t = [0;0;0.425];
    torso_const.vis.links(1).param = struct('radius',0.12,'height',0.85);
    torso_const.vis.links(1).props = link_props;
    
    torso_const.vis.links(2).handle = @createCylinder;
    torso_const.vis.links(2).R = rot(x0,18*pi/180);
    torso_const.vis.links(2).t = rot(x0,18*pi/180)*0.145*z0;
    torso_const.vis.links(2).param = struct('radius',0.12,'height',0.19);
    torso_const.vis.links(2).props = link_props;
    
    torso_const.vis.frame = struct('scale',0.15,'width',0.01);
    
    %%% Left Arm
    left_arm_const.name = 'sda10d_left_arm';
    % Kinematics
    left_arm_const.kin.H = R0*[x0 y0 x0 y0 z0 y0 z0];
    left_arm_const.kin.P = R0*[[0.05;-.1;.3] .215*x0 ...
                        zed .36*x0 .36*z0 zed .155*z0 zed];
    left_arm_const.kin.P(:,1) = t0 + left_arm_const.kin.P(:,1);
    left_arm_const.kin.joint_type = zeros(7,1);
    
    % Limits
    left_arm_const.limit.upper_joint_limit = upper_joint_limit;
    left_arm_const.limit.lower_joint_limit = lower_joint_limit;
    left_arm_const.limit.velocity_limit = velocity_limit;
    
    % Visualization parameters
    left_arm_const.vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        left_arm_const.vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        left_arm_const.vis.joints(n).props = joint_props;
    end
    
    left_arm_const.vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    left_arm_const.vis.links(2).handle = @createCylinder;
    left_arm_const.vis.links(2).R = rot(y0,pi/2);
    left_arm_const.vis.links(2).t = .0925*x0;
    left_arm_const.vis.links(2).param = struct('radius',0.08, ...
                                                'height',0.085);
    left_arm_const.vis.links(2).props = link_props;
    
    left_arm_const.vis.links(4).handle = @createCylinder;
    left_arm_const.vis.links(4).R = rot(y0,pi/2);
    left_arm_const.vis.links(4).t = .19*x0;
    left_arm_const.vis.links(4).param = struct('radius',0.06, ...
                                                'height',0.22);
    left_arm_const.vis.links(4).props = link_props;
    
    left_arm_const.vis.links(5).handle = @createCylinder;
    left_arm_const.vis.links(5).R = eye(3);
    left_arm_const.vis.links(5).t = .185*z0;
    left_arm_const.vis.links(5).param = struct('radius',0.05, ...
                                                'height',0.25);
    left_arm_const.vis.links(5).props = link_props;
    
    left_arm_const.vis.links(7).handle = @createCylinder;
    left_arm_const.vis.links(7).R = eye(3);
    left_arm_const.vis.links(7).t = .0975*z0;
    left_arm_const.vis.links(7).param = struct('radius',0.045, ...
                                                'height',0.095);
    left_arm_const.vis.links(7).props = link_props;
    
    left_arm_const.vis.frame = struct('scale',0.15,'width',0.01);
    
    %%% Right arm
    right_arm_const.name = 'sda10d_right_arm';
    
    % Kinematics
    right_arm_const.kin.H = R0*[x0 y0 x0 y0 z0 y0 z0];
    right_arm_const.kin.P = R0*[[-0.05;-.1;.3] -.215*x0 ...
                        zed -.36*x0 .36*z0 zed .155*z0 zed];
    right_arm_const.kin.P(:,1) = t0 + right_arm_const.kin.P(:,1);
    right_arm_const.kin.joint_type = zeros(7,1);
    
    % Limits
    right_arm_const.limit.upper_joint_limit = upper_joint_limit;
    right_arm_const.limit.lower_joint_limit = lower_joint_limit;
    right_arm_const.limit.velocity_limit = velocity_limit;
    
    % Visualization parameters
    right_arm_const.vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        right_arm_const.vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        right_arm_const.vis.joints(n).props = joint_props;
    end
    
    right_arm_const.vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    right_arm_const.vis.links(2).handle = @createCylinder;
    right_arm_const.vis.links(2).R = rot(y0,pi/2);
    right_arm_const.vis.links(2).t = -.0925*x0;
    right_arm_const.vis.links(2).param = struct('radius',0.08, ...
                                                'height',0.085);
    right_arm_const.vis.links(2).props = link_props;
    
    right_arm_const.vis.links(4).handle = @createCylinder;
    right_arm_const.vis.links(4).R = rot(y0,pi/2);
    right_arm_const.vis.links(4).t = -.19*x0;
    right_arm_const.vis.links(4).param = struct('radius',0.06, ...
                                                'height',0.22);
    right_arm_const.vis.links(4).props = link_props;
    
    right_arm_const.vis.links(5).handle = @createCylinder;
    right_arm_const.vis.links(5).R = eye(3);
    right_arm_const.vis.links(5).t = .185*z0;
    right_arm_const.vis.links(5).param = struct('radius',0.05, ...
                                                'height',0.25);
    right_arm_const.vis.links(5).props = link_props;
    
    right_arm_const.vis.links(7).handle = @createCylinder;
    right_arm_const.vis.links(7).R = eye(3);
    right_arm_const.vis.links(7).t = .0975*z0;
    right_arm_const.vis.links(7).param = struct('radius',0.045, ...
                                                'height',0.095);
    right_arm_const.vis.links(7).props = link_props;
    
    right_arm_const.vis.frame = struct('scale',0.15,'width',0.01);
     

    
    
    sda10_const.torso_const = torso_const;
    sda10_const.left_arm_const = left_arm_const;
    sda10_const.right_arm_const = right_arm_const;
end