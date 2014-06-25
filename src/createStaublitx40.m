function h_staubli = createStaublitx40(varargin)
    %
    % h_staubli = createStaublitx40(...)
    %
    % File to create a Staubli tx40 drawing object.
    %
    %
    % Optional properties for initialization are:
    %       'CreateFrames'      default: 'off'
    %       'Origin'            default: [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % depends on the following drawing package files:
    %       createCylinder.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %
    % returns handle to the staubli tx40 robot structure
    
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
    
    %%%%%% Define Kinematics for arm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    staubli.H = [z0 x0 x0 z0 x0 z0];
    staubli.P = [.32*z0 zed [.035;0;.225] zed .225*z0 .065*z0 zed];
    staubli.type = zeros(1,6);
    staubli.n = 6;
    staubli.origin = origin;
    
    %%%%%% Define visual properties for robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', [.9;0.9;0.9], ...
                    'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0.9;0.9;0.9]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    staubli.joint(1).radius = 0.084;
    staubli.joint(1).height = 0.165;
    staubli.joint(1).props = joint_props;
    staubli.joint(2).radius = 0.082;
    staubli.joint(2).height = 0.1855;
    staubli.joint(2).props = joint_props;
    staubli.joint(3).radius = 0.061;
    staubli.joint(3).height = 0.117;
    staubli.joint(3).props = joint_props;
    staubli.joint(4).radius = 0.0585;
    staubli.joint(4).height = 0.124;
    staubli.joint(4).props = joint_props;
    staubli.joint(5).radius = 0.04;
    staubli.joint(5).height = 0.093;
    staubli.joint(5).props = joint_props;
    staubli.joint(6).radius = 0.02;
    staubli.joint(6).height = 0.005;
    staubli.joint(6).props = joint_props;
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    staubli.link_type = [1 0 1 0 1 1 0];
    
    staubli.link(1).radius = 0.084;
    staubli.link(1).height = 0.230;
    staubli.link(1).t0 = .115*z0;
    staubli.link(1).R0 = eye(3);
    staubli.link(1).props = link_props;
    
    staubli.link(3).radius = 0.072;
    staubli.link(3).height = 0.368;
    staubli.link(3).t0 = [.1655;0;.1125];
    staubli.link(3).R0 = eye(3);
    staubli.link(3).props = link_props;
    
    staubli.link(5).radius = 0.0585;
    staubli.link(5).height = 0.123;
    staubli.link(5).t0 = .1235*z0;
    staubli.link(5).R0 = eye(3);
    staubli.link(5).props = link_props;
    
    staubli.link(6).radius = 0.02;
    staubli.link(6).height = 0.0225;
    staubli.link(6).t0 = 0.0513*z0;
    staubli.link(6).R0 = eye(3);
    staubli.link(6).props = link_props;
    
    staubli.frame.scale = 0.1;
    staubli.frame.width = 0.01;
    
    % Guess
    staubli.gripper.width = .1;
    staubli.gripper.height = .1;
    staubli.gripper.R0 = eye(3);
        
    h_staubli = createRobot(staubli,'CreateFrames',cf, ...
                                        'CreateGripper','on');
    axis equal;
end