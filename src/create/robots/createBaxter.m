    function handle = createBaxter(R0, t0, varargin)
    % 
    % h_baxter = createBaxter(R0, t0, ...)
    %
    % File to create a Baxter drawing object.  The Baxter robot 
    % consists of three serial chain robots representing the two arms and 
    % the head.  The base frame for the entire system is located at the
    % center of the waist between the body and the pedestal.
    %
    %
    %
    % Optional parameters for initialization are:
    %       'CreatePedestal'    default: 'on'
    %       'CreateFrames'      default: 'off'
    %
    % depends on the following drawing package files:
    %       defineBaxter.m
    %       createCuboid.m
    %       createCylinder.m
    %       createPrism.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %
    % returns handle to the baxter structure containing three branched 
    % robots:
    %       left_arm
    %       right_arm 
    %       head
    %
    % see also DEFINEBAXTER
    
    y0 = [0;1;0]; z0 = [0;0;1];
    
    % Walk through varargin
    for i=1:2:(nargin-2)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        elseif strcmp(varargin{i},'CreatePedestal')
            cp = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end
    if ~exist('cp','var'); cp = 'on'; end
    
    baxter = defineBaxter();
    left_arm = baxter.left_arm;
    right_arm = baxter.right_arm;
    head = baxter.head;
    
    %%%%%% Define visual properties for links and joints %%%%%%%%%%%%%%%%%%
    link_type1_props = {'FaceColor', [.9;0;0], ...
                            'EdgeAlpha', 0};
    link_type2_props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeAlpha', 0};
    joint_type1_props = {'FaceColor', [0.2;0.2;0.2]};
    joint_type2_props = {'FaceColor', [0.9;0;0]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define dimensions of each joint. 
    for i=1:length(left_arm.joint_type)
        left_arm.joints(i).radius = left_arm.joint_radius(i);
        left_arm.joints(i).height = left_arm.joint_height(i);
    end
    left_arm.joints(1).props = joint_type1_props;
    left_arm.joints(2).props = joint_type2_props;
    left_arm.joints(3).props = joint_type2_props;
    left_arm.joints(4).props = joint_type1_props;
    left_arm.joints(5).props = joint_type2_props;
    left_arm.joints(6).props = joint_type1_props;
    left_arm.joints(7).props = joint_type1_props;
    % Right arm is identical to left arm
    right_arm.joints = left_arm.joints;
    
    head.joints(1).radius = 0.06;
    head.joints(1).height = 0.08;
    head.joints(1).props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    left_arm.link_type = [0 1 0 1 0 1 0 1];
    right_arm.link_type = left_arm.link_type;
    head.link_type = [2 2];
    
    % Shoulder
    left_arm.links(2).radius = 0.075;
    left_arm.links(2).height = 0.2553;
    left_arm.links(2).t0 = [0;0;0.1777];
    left_arm.links(2).R0 = eye(3);
    left_arm.links(2).props = link_type2_props;
        
    % Upper Arm
    left_arm.links(4).radius = 0.075;
    left_arm.links(4).height = .2193;
    left_arm.links(4).t0 = rot(z0,pi/4)*[.1847;0;0];
    left_arm.links(4).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.links(4).props = link_type1_props;
    
    % Lower Arm
    left_arm.links(6).radius = 0.07;
    left_arm.links(6).height = .2443;
    left_arm.links(6).t0 = rot(z0,pi/4)*[.1921;0;0];
    left_arm.links(6).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.links(6).props = link_type1_props;
    
    % Wrist
    left_arm.links(8).radius = 0.05;
    left_arm.links(8).height = .1695;
    left_arm.links(8).t0 = rot(z0,pi/4)*[.1448;0;0];
    left_arm.links(8).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.links(8).props = link_type2_props;
    
    right_arm.links = left_arm.links;
    right_arm.links(4).t0 = rot(z0,-pi/4)*[.1847;0;0];
    right_arm.links(4).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm.links(6).t0 = rot(z0,-pi/4)*[.1921;0;0];
    right_arm.links(6).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm.links(8).t0 = rot(z0,-pi/4)*[.1448;0;0];
    right_arm.links(8).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    
    % Torso
    head.links(1).width = 0.33;
    head.links(1).length = 0.31;
    head.links(1).height = 0.52;
    head.links(1).t0 = [-.01;0;.26];
    head.links(1).R0 = eye(3);
    head.links(1).props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeColor', [0;0;0], ...
                            'EdgeAlpha', 1};
    
    % Screen
    head.links(2).width = 0.3;
    head.links(2).length = 0.02;
    head.links(2).height = 0.20;
    head.links(2).t0 = [.1039;0;-0.0038];
    head.links(2).R0 = rot(y0,pi/9)*rot(z0,pi/2);
    head.links(2).props = {'FaceColor', [0.9;0;0],...
                            'EdgeColor',[0.5;0.5;0.5]};
       
    %%%%%% Define reference frame dimensions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    left_arm.frame.scale = 0.2;
    left_arm.frame.width = 0.01;
    right_arm.frame = left_arm.frame;
    head.frame = right_arm.frame;
    
    %%%%%% Create robot objects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h_left_arm = createRobot(R0, t0, left_arm,'CreateFrames',cf);
    h_right_arm = createRobot(R0, t0, right_arm,'CreateFrames',cf);
    h_head = createRobot(R0, t0, head,'CreateFrames',cf);
    
    %%%%%% Attach extra body parts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Neck
    neck_props = {'FaceColor',[0.9;0;0],'EdgeAlpha',0};
    neck_params.radius = 0.04;
    neck_params.height = 0.28;
    t_neck = h_head.robots.frames(end).t - R0*[.1395;0;0.06];
    h_neck = createCylinder(R0*rot(y0,pi/8), t_neck, ...
                            neck_params, neck_props{:});
    h_neck.labels = attachPrefix('neck_',h_neck.labels);
    
    % Sonar head
    sonar_head_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    sonar_head_params.radius = 0.075;
    sonar_head_params.radius2 = 0.06;
    sonar_head_params.height = 0.16;
    t_sonar_head = h_head.robots.frames(end).t + R0*0.08*z0;
    h_sonar_head = createCylinder(R0, t_sonar_head, ...
                            sonar_head_params,sonar_head_props{:});
    h_sonar_head.labels = attachPrefix('sonar_head_',h_sonar_head.labels);
    
    % Arm Mounts
    arm_mount_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    arm_mount_param.width = 0.2;
    arm_mount_param.length = 0.15;
    arm_mount_param.height = 0.05;
    R_l = R0*rot(z0,75*pi/180);
    t_larm_mount = h_left_arm.robots.frames(1).t + R_l*[-.025;0;-0.075];
    h_left_arm_mount = createCuboid(R_l, t_larm_mount, ...
                            arm_mount_param, arm_mount_props{:});
    h_left_arm_mount.labels = attachPrefix('arm_mount_', ...
                                        h_left_arm_mount.labels);
    R_r = R0*rot(z0,-75*pi/180);
    t_rarm_mount = h_right_arm.robots.frames(1).t + R_r*[-.025;0;-0.075];
    h_right_arm_mount = createCuboid(R_r, t_rarm_mount, ...
                            arm_mount_param,arm_mount_props{:});
    h_right_arm_mount.labels = attachPrefix('arm_mount_', ...
                                        h_right_arm_mount.labels);
                                    
    % Attach parts to various robots
    h_head = attachObjectToRobot(h_neck,'base',h_head);
    h_head = attachObjectToRobot(h_sonar_head,'base',h_head);
    h_left_arm = attachObjectToRobot(h_left_arm_mount,'base', h_left_arm);
    h_right_arm = attachObjectToRobot(h_right_arm_mount,'base', h_right_arm);
    
    % Create optional pedestal
    if strcmp(cp,'on')
        % Pedestal top
        pedestal_top_props = {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 0};
        pedestal_top_param.radius = .18;
        pedestal_top_param.height = .12;
        t_pedestal_top = t0 - R0*.06*z0;
        h_pedestal_top = createCylinder(R0, t_pedestal_top, ...
                            pedestal_top_param,pedestal_top_props{:});

        % Pedestal body
        pedestal_body_props = {'FaceColor', [.2;.2;.2], 'EdgeAlpha', 0};
        pedestal_body_param.radius = .1;
        pedestal_body_param.height = .5;
        t_pedestal_body = t0 - R0*.37*z0;
        h_pedestal_body = createCylinder(R0, t_pedestal_body, ...
                            pedestal_body_param,pedestal_body_props{:});

        % Pedestal base
        pedestal_base_props = {'FaceColor', [.4;.4;.4], 'EdgeAlpha', 1};
        pedestal_base_param.height = .06;
        pedestal_base_param.polygon = ...
                    [-.1 .4 .4 .15 .15 .4 .4 -.1 -.5 -.5 -.3 -.3 -.5 -.5; ...
                    .2 .4 .33 .1 -.1 -.33 -.4 -.2 -.4 -.27 -.1 .1 .27 .4];
        t_pedestal_base = t0 - R0*.65*z0;
        h_pedestal_base = createPrism(R0, t_pedestal_base, ...
                            pedestal_base_param, pedestal_base_props{:});
            
        h_head = attachObjectToRobot(h_pedestal_top,'base',h_head);
        h_head = attachObjectToRobot(h_pedestal_body,'base',h_head);
        h_head = attachObjectToRobot(h_pedestal_base,'base',h_head);
    end
    
    %%%%%% Combine into single multibranch robot handle %%%%%%%%%%%%%%%%%%%
    handle = branchRobots(h_left_arm, h_right_arm);
    handle = branchRobots(handle, h_head);
end
    