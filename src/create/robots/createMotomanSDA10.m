function handle = createMotomanSDA10(R0, t0, varargin)
    %
    % handle = createMotomanSDA10(R0, t0, ...)
    %
    % Function to create a Yaskawa Motoman SDA10 robot drawing object.
    %
    % Optional parameters for initialization are:
    %       'CreateFrames'      default: 'off'
    %
    % depends on the following drawing package files:
    %       createCylinder.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %       branchRobots.m
    %
    % returns handle to the Motoman SDA10 robot structure - contains:
    %       left_arm
    %       right_arm
    %
    % see also DEFINEMOTOMANSDA10

    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; 
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end

    %%%%%% Define Kinematics for the arms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sda10_const = defineMotomanSDA10();
    torso = sda10_const.torso;
    left_arm = sda10_const.left_arm;
    right_arm = sda10_const.right_arm;
    
    %%%%%% Define visual properties for robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', [0.8;0.8;0.8], 'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0;0;1]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    torso.joints(1).radius = torso.joint_radius(1);
    torso.joints(1).height = torso.joint_height(1);
    torso.joints(1).props = joint_props;
    
    for i=1:numel(left_arm.joint_type)
        left_arm.joints(i).radius = left_arm.joint_radius(i);
        left_arm.joints(i).height = left_arm.joint_height(i);
        left_arm.joints(i).props = joint_props;
    end
    right_arm.joints = left_arm.joints;
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    torso.link_type = [1 1];
    left_arm.link_type = [0 1 0 1 1 0 1 0];
    right_arm.link_type = left_arm.link_type;
    
    torso.links(1).radius = 0.12;
    torso.links(1).height = 0.85;
    torso.links(1).t0 = [0;0;0.425];
    torso.links(1).R0 = eye(3);
    torso.links(1).props = link_props;
    
    torso.links(2).radius = 0.12;
    torso.links(2).height = 0.19;
    torso.links(2).t0 = rot(x0,18*pi/180)*(.19/2 + .05)*z0;
    torso.links(2).R0 = rot(x0,18*pi/180);
    torso.links(2).props = link_props;
    
    left_arm.links(2).radius = 0.08;
    left_arm.links(2).height = 0.085; 
    left_arm.links(2).t0 = .0925*x0; 
    left_arm.links(2).R0 = rot(y0,pi/2);
    left_arm.links(2).props = link_props;
    
    left_arm.links(4).radius = 0.06;
    left_arm.links(4).height = 0.22; 
    left_arm.links(4).t0 = .19*x0; 
    left_arm.links(4).R0 = rot(y0,pi/2);
    left_arm.links(4).props = link_props;
    
    left_arm.links(5).radius = 0.05;
    left_arm.links(5).height = 0.25; 
    left_arm.links(5).t0 = .185*z0; 
    left_arm.links(5).R0 = eye(3);
    left_arm.links(5).props = link_props;
    
    left_arm.links(7).radius = 0.045;
    left_arm.links(7).height = .095;
    left_arm.links(7).t0 = .0975*z0;
    left_arm.links(7).R0 = eye(3);
    left_arm.links(7).props = link_props;
    
    right_arm.links = left_arm.links;
    right_arm.links(2).t0 = -.0925*x0; 
    right_arm.links(2).R0 = rot(y0,pi/2);
    
    right_arm.links(4).t0 = -.19*x0; 
    right_arm.links(4).R0 = rot(y0,pi/2);
    
    right_arm.links(5).t0 = .185*z0; 
    right_arm.links(5).R0 = eye(3);
    
    right_arm.links(7).t0 = .0975*z0;
    right_arm.links(7).R0 = eye(3);
    
    left_arm.frame.scale = 0.15;
    left_arm.frame.width = 0.01;
    right_arm.frame = left_arm.frame;
        
    %%%%%% Build full robot structure %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Build individual robot structures
    h_torso = createRobot(R0, t0, torso,'CreateFrames',cf);
    h_left_arm = createRobot(R0, t0, left_arm,'CreateFrames',cf);
    h_left_arm = updateRigidBody(eye(3),sum(torso.P,2),h_left_arm);
    h_right_arm = createRobot(R0, t0, right_arm,'CreateFrames',cf);
    h_right_arm = updateRigidBody(eye(3),sum(torso.P,2),h_right_arm);
    
    h_arms = branchRobots(h_left_arm,h_right_arm);
    handle = combineRobots(h_torso,h_arms);
end