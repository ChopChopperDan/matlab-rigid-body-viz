function handle = createStaublitx40(varargin)
    %
    % handle = createStaublitx40(...)
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
    %
    % see also DEFINESTAUBLITX40
    
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
    
    staubli = defineStaublitx40(origin);
    
    %%%%%% Define visual properties for robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', [.9;0.9;0.9], ...
                    'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0.9;0.9;0.9]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:staubli.n
        staubli.joint(i).radius = staubli.joint_radius(i);
        staubli.joint(i).height = staubli.joint_height(i);
        staubli.joint(i).props = joint_props;
    end
    
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
            
    handle = createRobot(staubli,'CreateFrames',cf, ...
                                        'CreateGripper','off');
end