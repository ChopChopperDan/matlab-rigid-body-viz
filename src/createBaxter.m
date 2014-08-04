    function handle = createBaxter(varargin)
    % 
    % h_baxter = createBaxter(...)
    %
    % File to create a Baxter drawing object.  The Baxter robot 
    % consists of three serial chain robots representing the two arms and 
    % the head.  The base frame for the entire system is located between 
    % the two rear wheels, on the ground.
    %
    %
    % Optional parameters for initialization are:
    %       'CreatePedestal'    default: 'on'
    %       'CreateFrames'      default: 'off'
    %       'Origin'            default: [eye(3) [0;0;0]; [0 0 0] 1]
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
    % returns handle to the baxter structure containing three robot
    % structures:
    %       left_arm
    %       right_arm
    %       head
    %
    % see also DEFINEBAXTER
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    % Walk through varargin
    for i=1:2:(nargin-1)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        elseif strcmp(varargin{i},'CreatePedestal')
            cp = varargin{i+1};
        elseif strcmp(varargin{i},'Origin')
            origin = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end
    if ~exist('cp','var'); cp = 'on'; end
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end
    
    baxter = defineBaxter();
    left_arm = baxter.left_arm;
    left_arm.origin = origin;
    right_arm = baxter.right_arm;
    right_arm.origin = origin;
    head = baxter.head;
    head.origin = origin;
    
    %%%%%% Define visual properties for links and joints %%%%%%%%%%%%%%%%%%
    link_type1_props = {'FaceColor', [.9;0;0], ...
                            'EdgeAlpha', 0};
    link_type2_props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeAlpha', 0};
    joint_type1_props = {'FaceColor', [0.2;0.2;0.2]};
    joint_type2_props = {'FaceColor', [0.9;0;0]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define dimensions of each joint. 
    for i=1:left_arm.n
        left_arm.joint(i).radius = left_arm.joint_radius(i);
        left_arm.joint(i).height = left_arm.joint_height(i);
    end
    left_arm.joint(1).props = joint_type1_props;
    left_arm.joint(2).props = joint_type2_props;
    left_arm.joint(3).props = joint_type2_props;
    left_arm.joint(4).props = joint_type1_props;
    left_arm.joint(5).props = joint_type2_props;
    left_arm.joint(6).props = joint_type1_props;
    left_arm.joint(7).props = joint_type1_props;
    % Right arm is identical to left arm
    right_arm.joint = left_arm.joint;
    
    head.joint(1).radius = 0.06;
    head.joint(1).height = 0.08;
    head.joint(1).props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    left_arm.link_type = [0 1 0 1 0 1 0 1];
    right_arm.link_type = left_arm.link_type;
    head.link_type = [2 2];
    
    % Shoulder
    left_arm.link(2).radius = 0.075;
    left_arm.link(2).height = 0.2553;
    left_arm.link(2).t0 = [0;0;0.1777];
    left_arm.link(2).R0 = eye(3);
    left_arm.link(2).props = link_type2_props;
        
    % Upper Arm
    left_arm.link(4).radius = 0.075;
    left_arm.link(4).height = .2193;
    left_arm.link(4).t0 = rot(z0,pi/4)*[.1847;0;0];
    left_arm.link(4).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.link(4).props = link_type1_props;
    
    % Lower Arm
    left_arm.link(6).radius = 0.07;
    left_arm.link(6).height = .2443;
    left_arm.link(6).t0 = rot(z0,pi/4)*[.1921;0;0];
    left_arm.link(6).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.link(6).props = link_type1_props;
    
    % Wrist
    left_arm.link(8).radius = 0.05;
    left_arm.link(8).height = .1695;
    left_arm.link(8).t0 = rot(z0,pi/4)*[.1448;0;0];
    left_arm.link(8).R0 = rot(z0,pi/4)*rot(y0,pi/2);
    left_arm.link(8).props = link_type2_props;
    
    right_arm.link = left_arm.link;
    right_arm.link(4).t0 = rot(z0,-pi/4)*[.1847;0;0];
    right_arm.link(4).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm.link(6).t0 = rot(z0,-pi/4)*[.1921;0;0];
    right_arm.link(6).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    right_arm.link(8).t0 = rot(z0,-pi/4)*[.1448;0;0];
    right_arm.link(8).R0 = rot(z0,-pi/4)*rot(y0,pi/2);
    
    % Torso
    head.link(1).width = 0.33;
    head.link(1).length = 0.31;
    head.link(1).height = 0.52;
    head.link(1).t0 = [-.01;0;.26];
    head.link(1).R0 = eye(3);
    head.link(1).props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeColor', [0;0;0], ...
                            'EdgeAlpha', 1};
    
    % Screen
    head.link(2).width = 0.3;
    head.link(2).length = 0.02;
    head.link(2).height = 0.20;
    head.link(2).t0 = [.1039;0;-0.0038];
    head.link(2).R0 = rot(y0,pi/9)*rot(z0,pi/2);
    head.link(2).props = {'FaceColor', [0.9;0;0],...
                            'EdgeColor',[0.5;0.5;0.5]};
       
    %%%%%% Define reference frame dimensions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    left_arm.frame.scale = 0.2;
    left_arm.frame.width = 0.01;
    right_arm.frame = left_arm.frame;
    head.frame = right_arm.frame;
    
    %%%%%% Define gripper dimensions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    left_arm.gripper.aperture = 0.06;
    left_arm.gripper.stroke = 0.06;
    left_arm.gripper.height = 0.08;
    left_arm.gripper.fingerwidth = 0.01;
    left_arm.gripper.R0 = rot(z0,-pi/4)*rot(x0,-pi/2);
    right_arm.gripper = left_arm.gripper;
    right_arm.gripper.R0 = rot(z0,pi/4)*rot(x0,pi/2);
    
    %%%%%% Create robot objects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    handle.left_arm = createRobot(left_arm,'CreateFrames',cf, ...
                                            'CreateGripper','on');
    handle.right_arm = createRobot(right_arm,'CreateFrames',cf, ...
                                            'CreateGripper','on');
    handle.head = createRobot(head,'CreateFrames',cf, ...
                                            'CreateGripper','off');
    
    %%%%%% Attach extra body parts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Neck
    neck_props = {'FaceColor',[0.9;0;0],'EdgeAlpha',0};
    neck_params.radius = 0.04;
    neck_params.height = 0.28;
    t_neck = handle.head.frame(2).t - origin(1:3,1:3)*[.1395;0;0.06];
    h_neck = createCylinder(origin(1:3,1:3)*rot(y0,pi/8), ...
                        t_neck, neck_params, neck_props{:});
    h_neck.labels = attachPrefix('neck_',h_neck.labels);
    
    % Sonar head
    sonar_head_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    sonar_head_params.radius = 0.075;
    sonar_head_params.radius2 = 0.06;
    sonar_head_params.height = 0.16;
    t_sonar_head = handle.head.frame(2).t + origin(1:3,1:3)*0.08*z0;
    h_sonar_head = createCylinder(origin(1:3,1:3), ...
                    t_sonar_head, sonar_head_params,sonar_head_props{:});
    h_sonar_head.labels = attachPrefix('sonar_head_',h_sonar_head.labels);
    
    % Arm Mounts
    arm_mount_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    arm_mount_param.width = 0.2;
    arm_mount_param.length = 0.15;
    arm_mount_param.height = 0.05;
    t_larm_mount = handle.left_arm.frame(2).t + ...
            origin(1:3,1:3)*rot(z0,75*pi/180)*[-.025;0;-0.075];
    h_left_arm_mount = createCuboid(origin(1:3,1:3)*rot(z0,75*pi/180), ...
                t_larm_mount, arm_mount_param, arm_mount_props{:});
    h_left_arm_mount.labels = attachPrefix('arm_mount_', ...
                                        h_left_arm_mount.labels);
    
    t_rarm_mount = handle.right_arm.frame(2).t + ...
            origin(1:3,1:3)*rot(z0,-75*pi/180)*[-.025;0;-0.075];
    h_right_arm_mount = createCuboid(origin(1:3,1:3)*rot(z0,-75*pi/180), ...
                t_rarm_mount, arm_mount_param,arm_mount_props{:});
    h_right_arm_mount.labels = attachPrefix('arm_mount_', ...
                                        h_right_arm_mount.labels);
                                    
    % Attach parts to robot
    handle.head = attachObjectToRobot(h_neck,0,handle.head);
    handle.head = attachObjectToRobot(h_sonar_head,0,handle.head);
    handle.left_arm = attachObjectToRobot(h_left_arm_mount,0, ...
                                    handle.left_arm);
    handle.right_arm = attachObjectToRobot(h_right_arm_mount,0, ...
                                    handle.right_arm);
    
    % Create optional pedestal
    if strcmp(cp,'on')
        % Pedestal top
        pedestal_top_props = {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 0};
        pedestal_top_param.radius = .18;
        pedestal_top_param.height = .12;
        t_pedestal_top = handle.head.frame(1).t - ...
                            handle.head.frame(1).R*.06*z0;
        h_pedestal_top = createCylinder(handle.head.frame(1).R, ...
                t_pedestal_top, pedestal_top_param,pedestal_top_props{:});

        % Pedestal body
        pedestal_body_props = {'FaceColor', [.2;.2;.2], 'EdgeAlpha', 0};
        pedestal_body_param.radius = .1;
        pedestal_body_param.height = .5;
        t_pedestal_body = handle.head.frame(1).t - origin(1:3,1:3)*.37*z0;
        h_pedestal_body = createCylinder(origin(1:3,1:3), ...
                t_pedestal_body, pedestal_body_param,pedestal_body_props{:});

        % Pedestal base
        pedestal_base_props = {'FaceColor', [.4;.4;.4], 'EdgeAlpha', 1};
        pedestal_base_param.height = .06;
        pedestal_base_param.polygon = ...
                    [-.1 .4 .4 .15 .15 .4 .4 -.1 -.5 -.5 -.3 -.3 -.5 -.5; ...
                    .2 .4 .33 .1 -.1 -.33 -.4 -.2 -.4 -.27 -.1 .1 .27 .4];
        t_pedestal_base = handle.head.frame(1).t - origin(1:3,1:3)*.65*z0;
        h_pedestal_base = createPrism(origin(1:3,1:3), ...
                t_pedestal_base, pedestal_base_param, pedestal_base_props{:});
            
        handle.head = attachObjectToRobot(h_pedestal_top,0,handle.head);
        handle.head = attachObjectToRobot(h_pedestal_body,0,handle.head);
        handle.head = attachObjectToRobot(h_pedestal_base,0,handle.head);
    end
    
end
    