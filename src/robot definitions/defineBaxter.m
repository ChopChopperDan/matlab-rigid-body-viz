function baxter_const = defineBaxter(varargin)
    %
    % baxter_const = defineBaxter()
    % baxter_const = defineBaxter(...) - allows additional optional
    %                                    parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %       'Pedestal'      :   'on'/'off' (default on)
    %
    % define-file for the Rethink Robotics Baxter.  Returns struct with the
    % following form:
    %
    % root
    %   -> left_arm_const
    %       -> name                 : 'baxter_left_arm'
    %       -> kin
    %           -> H                : [3 x 7] joint axes
    %           -> P                : [3 x 8] inter-joint translation
    %           -> joint_type       : [1 x 7] joint types
    %       -> limit
    %           -> upper_joint_limit :  [7 x 1] upper joint limits [rad]
    %           -> lower_joint_limit :  [7 x 1] lower joint limits [rad]
    %           -> velocity_limit    :  [7 x 1] velocity limits    [rad/s]
    %           -> torque_limit      :  [7 x 1] effort limits      [Nm/s]
    %       -> vis
    %           -> joints      :  [7 x 1] struct array of joint definitions
    %           -> links       :  [8 x 1] struct array of link definitions
    %           -> frame       :  3D frame dimensions
    %           -> peripherals :  struct array (arm mount)
    %   -> right_arm_const
    %       -> name                 : 'baxter_right_arm'
    %       -> kin
    %           -> H                : [3 x 7] joint axes
    %           -> P                : [3 x 8] inter-joint translation
    %           -> joint_type       : [1 x 7] joint types
    %       -> limit
    %           -> upper_joint_limit :  [7 x 1] upper joint limits [rad]
    %           -> lower_joint_limit :  [7 x 1] lower joint limits [rad]
    %           -> velocity_limit    :  [7 x 1] velocity limits    [rad/s]
    %           -> torque_limit      :  [7 x 1] effort limits      [Nm/s]
    %       -> vis
    %           -> joints      :  [7 x 1] struct array of joint definitions
    %           -> links       :  [8 x 1] struct array of link definitions
    %           -> frame       :  3D frame dimensions
    %           -> peripherals :  struct array (arm mount)
    %   -> head_const
    %       -> name                 : 'baxter_head'
    %       -> kin
    %           -> H                : [3 x 1] joint axes
    %           -> P                : [3 x 2] inter-joint translation
    %           -> joint_type       : [1 x 1] joint types
    %       -> limit
    %           -> upper_joint_limit :  [1 x 1] upper joint limits [rad]
    %           -> lower_joint_limit :  [1 x 1] lower joint limits [rad]
    %       -> vis
    %           -> joints      :  [1 x 1] struct array
    %           -> links       :  [2 x 1] height of each joint [m]
    %           -> frame       :  3D frame dimensions
    %           -> peripherals :  struct array 
    %                                   (sonar head, neck, [opt] pedestal)
    %
    %   see also CREATEBAXTER 
    
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    flags = {'Origin', 'Pedestal'};
    defaults = {[eye(3) zed; zed' 1], 'on'};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    cp = opt_values{2};

    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    joint_radius = [.075;.075;.075;.07;.07;.06;.06];
    joint_height = [.1;.19;.15;.18;.14;.15;.12];
    
    upper_joint_limit = [97.5; 60; 175; 150; 175; 120; 175]*pi/180;
    lower_joint_limit = [-97.5;-123; -175; -2.5; -175; -90; -175]*pi/180;
    velocity_limit = [2.5;2.5;2.5;2.5;5;5;5];
    torque_limit = [60;60;60;60;20;20;20];
    
    link_type1_props = {'FaceColor', [.9;0;0], ...
                            'EdgeAlpha', 0};
    link_type2_props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeAlpha', 0};
    joint_type1_props = {'FaceColor', [0.2;0.2;0.2]};
    joint_type2_props = {'FaceColor', [0.9;0;0]};
    
    arm_mount_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    arm_mount_param = struct('width',0.2,'length',0.15,'height',0.05);
    
    %%% Left Arm
    % Kinematic Constants
    left_arm_const.name = 'baxter_left_arm';
    left_arm_const.kin.H = R0*rot(z0,pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    left_arm_const.kin.P = R0*[[0.06375;.25888;0.119217], ...
                            rot(z0,pi/4)*[[0.069;0;0.27035], ...
                            zed, [0.36435;0;-0.069], ...
                            zed, [0.37429;0;-0.01], ...
                            zed, 0.229525*x0]];
    left_arm_const.kin.P(:,1) = t0 + left_arm_const.kin.P(:,1);
    left_arm_const.kin.joint_type = zeros(1,7);
    
    % Dynamic Limits
    left_arm_const.limit.upper_joint_limit = upper_joint_limit;
    left_arm_const.limit.lower_joint_limit = lower_joint_limit;
    left_arm_const.limit.velocity_limit = velocity_limit;
    left_arm_const.limit.torque_limit = torque_limit;
    
    % Visualization definitions
    left_arm_const.vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        left_arm_const.vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        left_arm_const.vis.joints(n).props = joint_type1_props;
    end
    left_arm_const.vis.joints(2).props = joint_type2_props;
    left_arm_const.vis.joints(3).props = joint_type2_props;
    left_arm_const.vis.joints(5).props = joint_type2_props;
    
    left_arm_const.vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    left_arm_const.vis.links(2).handle = @createCylinder;
    left_arm_const.vis.links(2).R = eye(3);
    left_arm_const.vis.links(2).t = [0;0;0.1777];
    left_arm_const.vis.links(2).param = struct('radius',0.075,'height',0.2553);
    left_arm_const.vis.links(2).props = link_type2_props;
    
    left_arm_const.vis.links(4).handle = @createCylinder;
    left_arm_const.vis.links(4).R = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm_const.vis.links(4).t = rot(z0,pi/4)*[.1847;0;0];
    left_arm_const.vis.links(4).param = struct('radius',0.075,'height',.2193);
    left_arm_const.vis.links(4).props = link_type1_props;
    
    left_arm_const.vis.links(6).handle = @createCylinder;
    left_arm_const.vis.links(6).R = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm_const.vis.links(6).t = rot(z0,pi/4)*[.1921;0;0];
    left_arm_const.vis.links(6).param = struct('radius',0.07,'height',.2443);
    left_arm_const.vis.links(6).props = link_type1_props;
        
    left_arm_const.vis.links(8).handle = @createCylinder;
    left_arm_const.vis.links(8).R = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm_const.vis.links(8).t = rot(z0,pi/4)*[.1448;0;0];
    left_arm_const.vis.links(8).param = struct('radius',0.05,'height',.1695);
    left_arm_const.vis.links(8).props = link_type2_props;
    
    left_arm_const.vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral arm mount
    left_arm_const.vis.peripherals.id = 'arm_mount';
    left_arm_const.vis.peripherals.frame = 'base';
    left_arm_const.vis.peripherals.handle = @createCuboid;
    left_arm_const.vis.peripherals.R = R0*rot(z0,75*pi/180);
    left_arm_const.vis.peripherals.t = ...
                            t0 + R0*([0.06375;.25888;0.119217] - ...
                                    rot(z0,75*pi/180)*[0.025;0;0.075]);
    left_arm_const.vis.peripherals.param = arm_mount_param;
    left_arm_const.vis.peripherals.props = arm_mount_props;
    
    %%% Right Arm
    % Kinematic Constants
    right_arm_const.name = 'baxter_right_arm';
    right_arm_const.kin.H = R0*rot(z0,-pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    right_arm_const.kin.P = R0*[[0.06375;-.25888;0.119217], ...
                            rot(z0,-pi/4)*[[0.069;0;0.27035], ...
                            zed, [0.36435;0;-0.069], ...
                            zed, [0.37429;0;-0.01], ...
                            zed, 0.229525*x0]];
    right_arm_const.kin.P(:,1) = t0 + right_arm_const.kin.P(:,1);
    right_arm_const.kin.joint_type = zeros(1,7);
    
    
    % Dynamic Limits
    right_arm_const.limit.upper_joint_limit = upper_joint_limit;
    right_arm_const.limit.lower_joint_limit = lower_joint_limit;
    right_arm_const.limit.velocity_limit = velocity_limit;
    right_arm_const.limit.torque_limit = torque_limit;
    
    % Visualization definitions
    right_arm_const.vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        right_arm_const.vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        right_arm_const.vis.joints(n).props = joint_type1_props;
    end
    right_arm_const.vis.joints(2).props = joint_type2_props;
    right_arm_const.vis.joints(3).props = joint_type2_props;
    right_arm_const.vis.joints(5).props = joint_type2_props;
    
    right_arm_const.vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    right_arm_const.vis.links(2).handle = @createCylinder;
    right_arm_const.vis.links(2).R = eye(3);
    right_arm_const.vis.links(2).t = [0;0;0.1777];
    right_arm_const.vis.links(2).param = struct('radius',0.075,'height',0.2553);
    right_arm_const.vis.links(2).props = link_type2_props;
    
    right_arm_const.vis.links(4).handle = @createCylinder;
    right_arm_const.vis.links(4).R = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm_const.vis.links(4).t = rot(z0,-pi/4)*[.1847;0;0];
    right_arm_const.vis.links(4).param = struct('radius',0.075,'height',.2193);
    right_arm_const.vis.links(4).props = link_type1_props;
    
    right_arm_const.vis.links(6).handle = @createCylinder;
    right_arm_const.vis.links(6).R = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm_const.vis.links(6).t = rot(z0,-pi/4)*[.1921;0;0];
    right_arm_const.vis.links(6).param = struct('radius',0.07,'height',.2443);
    right_arm_const.vis.links(6).props = link_type1_props;
        
    right_arm_const.vis.links(8).handle = @createCylinder;
    right_arm_const.vis.links(8).R = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm_const.vis.links(8).t = rot(z0,-pi/4)*[.1448;0;0];
    right_arm_const.vis.links(8).param = struct('radius',0.05,'height',.1695);
    right_arm_const.vis.links(8).props = link_type2_props;
    
    right_arm_const.vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral arm mount
    right_arm_const.vis.peripherals.id = 'arm_mount';
    right_arm_const.vis.peripherals.frame = 'base';
    right_arm_const.vis.peripherals.handle = @createCuboid;
    right_arm_const.vis.peripherals.R = R0*rot(z0,-75*pi/180);
    right_arm_const.vis.peripherals.t = ...
                            t0 + R0*([0.06375;-.25888;0.119217] - ...
                                    rot(z0,-75*pi/180)*[0.025;0;0.075]);
    right_arm_const.vis.peripherals.param = arm_mount_param;
    right_arm_const.vis.peripherals.props = arm_mount_props;
    
    
    %%% Head
    % Kinematic Constants
    head_const.name = 'baxter_head';
    head_const.kin.H = R0*z0;
    head_const.kin.P = R0*[[.0599;0;.6955] zed];
    head_const.kin.P(:,1) = t0 + head_const.kin.P(:,1);
    head_const.kin.joint_type = 0;
    
    % Dynamic Limits
    head_const.limit.upper_joint_limit = pi/2;
    head_const.limit.lower_joint_limit = -pi/2;
    
    % Visualization definition
    head_const.vis.joints(1).param = struct('radius',0.06,'height',0.08);
    head_const.vis.joints(1).props = {'FaceColor',[0.2;0.2;0.2], ...
                                        'EdgeAlpha',0};
    
    head_const.vis.links(1).handle = @createCuboid;
    head_const.vis.links(1).R = eye(3);
    head_const.vis.links(1).t = [-.01;0;.26];
    head_const.vis.links(1).param = ...
                    struct('width',0.33, 'length',0.31,'height',.52);
    head_const.vis.links(1).props = {'FaceColor', [.2;0.2;0.2], ...
                                    'EdgeColor', [0;0;0], ...
                                    'EdgeAlpha', 1};

    head_const.vis.links(2).handle = @createCuboid;
    head_const.vis.links(2).R = rot(y0,pi/9)*rot(z0,pi/2);
    head_const.vis.links(2).t = [.1039;0;-0.0038];
    head_const.vis.links(2).param = ...
                    struct('width',0.3, 'length',0.02,'height',.2);
    head_const.vis.links(2).props = {'FaceColor', [0.9;0;0],...
                                    'EdgeColor',[0.5;0.5;0.5]};
    
    head_const.vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral head and neck
    head_const.vis.peripherals(1).id = 'sonar_head';
    head_const.vis.peripherals(1).frame = 'tool';
    head_const.vis.peripherals(1).handle = @createCylinder;
    head_const.vis.peripherals(1).R = R0;
    head_const.vis.peripherals(1).t = R0*0.08*z0;
    head_const.vis.peripherals(1).param = struct('radius', 0.075, ...
                                              'radius2', 0.06, ...
                                              'height', 0.16);
    head_const.vis.peripherals(1).props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    
    head_const.vis.peripherals(2).id = 'neck';
    head_const.vis.peripherals(2).frame = 'tool';
    head_const.vis.peripherals(2).handle = @createCylinder;
    head_const.vis.peripherals(2).R = R0*rot(y0,pi/8);
    head_const.vis.peripherals(2).t = -R0*[.1395;0;0.06];
    head_const.vis.peripherals(2).param = struct('radius', 0.04, 'height', 0.28);
    head_const.vis.peripherals(2).props = {'FaceColor',[0.9;0;0],'EdgeAlpha',0};
    
    % Pedestal (optional)
    if strcmp(cp,'on')
        head_const.vis.peripherals(3).id = 'pedestal_top';
        head_const.vis.peripherals(3).frame = 'base';
        head_const.vis.peripherals(3).handle = @createCylinder;
        head_const.vis.peripherals(3).R = R0;
        head_const.vis.peripherals(3).t = t0 - R0*.06*z0;
        head_const.vis.peripherals(3).param = struct('radius', 0.18, ...
                                                    'height', 0.12);
        head_const.vis.peripherals(3).props = ...
                            {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 0};
        
        head_const.vis.peripherals(4).id = 'pedestal_body';
        head_const.vis.peripherals(4).frame = 'base';
        head_const.vis.peripherals(4).handle = @createCylinder;
        head_const.vis.peripherals(4).R = R0;
        head_const.vis.peripherals(4).t = t0 - R0*.37*z0;
        head_const.vis.peripherals(4).param = struct('radius', 0.1, ...
                                                    'height', 0.5);
        head_const.vis.peripherals(4).props = ...
                            {'FaceColor', [0.2;0.2;0.2], 'EdgeAlpha', 0};
                        
        base_shape = [-.1 .4 .4 .15 .15 .4 .4 -.1 -.5 -.5 -.3 -.3 -.5 -.5; ...
                    .2 .4 .33 .1 -.1 -.33 -.4 -.2 -.4 -.27 -.1 .1 .27 .4];
        head_const.vis.peripherals(5).id = 'pedestal_base';
        head_const.vis.peripherals(5).frame = 'base';
        head_const.vis.peripherals(5).handle = @createPrism;
        head_const.vis.peripherals(5).R = R0;
        head_const.vis.peripherals(5).t = t0 - R0*.65*z0;
        head_const.vis.peripherals(5).param = struct('height', 0.06, ...
                                                    'polygon', base_shape);
        head_const.vis.peripherals(5).props = ...
                            {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 1};
    end
    
    baxter_const.left_arm_const = left_arm_const;
    baxter_const.right_arm_const = right_arm_const;
    baxter_const.head_const = head_const;
end