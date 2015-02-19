function handle = createMotomanSDA10(varargin)
    %
    % handle = createMotomanSDA10(...)
    %
    % Function to create a Yaskawa Motoman SDA10 robot drawing object.
    %
    % Optional parameters for initialization are:
    %       'CreateFrames'      default: 'off'
    %       'Origin'            default: [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % depends on the following drawing package files:
    %       createCylinder.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %
    % returns handle to the Motoman SDA10 robot structure - contains:
    %       left_arm
    %       right_arm
    %
    % see also DEFINEMOTOMANSDA10

    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    % Walk through varargin
    for i=1:2:(nargin-1)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        elseif strcmp(varargin{i},'Origin')
            origin = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end

    %%%%%% Define Kinematics for the arms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sda10_const = defineMotomanSDA10(origin);
    left_arm = sda10_const.left_arm;
    right_arm = sda10_const.right_arm;
    
    %%%%%% Define visual properties for robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', [0.8;0.8;0.8], ...
                    'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0;0;1]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:left_arm.n
        left_arm.joint(i).radius = left_arm.joint_radius(i);
        left_arm.joint(i).height = left_arm.joint_height(i);
        left_arm.joint(i).props = joint_props;
    end
    right_arm.joint = left_arm.joint;
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    left_arm.link_type = [1 1 1 0 1 1 0 1 0];
    right_arm.link_type = left_arm.link_type;
    right_arm.link_type(1:2) = 0;
    
    left_arm.link(1).radius = 0.12;
    left_arm.link(1).height = 0.85;
    left_arm.link(1).t0 = [0;0;0.425];
    left_arm.link(1).R0 = eye(3);
    left_arm.link(1).props = link_props;
    
    left_arm.link(2).radius = 0.12;
    left_arm.link(2).height = 0.244;
    left_arm.link(2).t0 = [0;-0.05;.175];
    left_arm.link(2).R0 = rot(x0,0.2783);
    left_arm.link(2).props = link_props;
    
    left_arm.link(3).radius = 0.08;
    left_arm.link(3).height = 0.085; %.115
    left_arm.link(3).t0 = .0925*x0; %.1075
    left_arm.link(3).R0 = rot(y0,pi/2);
    left_arm.link(3).props = link_props;
    
    left_arm.link(5).radius = 0.06;
    left_arm.link(5).height = 0.22; %.26
    left_arm.link(5).t0 = .19*x0; %.18
    left_arm.link(5).R0 = rot(y0,pi/2);
    left_arm.link(5).props = link_props;
    
    left_arm.link(6).radius = 0.05;
    left_arm.link(6).height = 0.25; %.26
    left_arm.link(6).t0 = .185*z0; %.18
    left_arm.link(6).R0 = eye(3);
    left_arm.link(6).props = link_props;
    
    left_arm.link(8).radius = 0.045;
    left_arm.link(8).height = .095;
    left_arm.link(8).t0 = .0975*z0;
    left_arm.link(8).R0 = eye(3);
    left_arm.link(8).props = link_props;
    
    right_arm.link(3).radius = 0.08;
    right_arm.link(3).height = 0.085; %.115
    right_arm.link(3).t0 = -.0925*x0; %.1075
    right_arm.link(3).R0 = rot(y0,pi/2);
    right_arm.link(3).props = link_props;
    
    right_arm.link(5).radius = 0.06;
    right_arm.link(5).height = 0.22; %.26
    right_arm.link(5).t0 = -.19*x0; %.18
    right_arm.link(5).R0 = rot(y0,pi/2);
    right_arm.link(5).props = link_props;
    
    right_arm.link(6).radius = 0.05;
    right_arm.link(6).height = 0.25; %.26
    right_arm.link(6).t0 = .185*z0; %.18
    right_arm.link(6).R0 = eye(3);
    right_arm.link(6).props = link_props;
    
    right_arm.link(8).radius = 0.045;
    right_arm.link(8).height = .095;
    right_arm.link(8).t0 = .0975*z0;
    right_arm.link(8).R0 = eye(3);
    right_arm.link(8).props = link_props;
    
    left_arm.frame.scale = 0.15;
    left_arm.frame.width = 0.01;
    right_arm.frame = left_arm.frame;
        
    %%%%%% Build full robot structure %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    handle.left_robot = createRobot(left_arm,'CreateFrames',cf, ...
                                                'CreateGripper','off');
    handle.right_robot = createRobot(right_arm,'CreateFrames',cf, ...
                                                'CreateGripper','off');
end