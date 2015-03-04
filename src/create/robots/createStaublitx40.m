function handle = createStaublitx40(R0, t0, varargin)
    %
    % handle = createStaublitx40(R0, t0, ...)
    %
    % File to create a Staubli tx40 drawing object.
    %
    %
    % Optional properties for initialization are:
    %       'CreateFrames'      default: 'off'
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
    
    z0 = [0;0;1];
    
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
    
    staubli = defineStaublitx40();
    
    %%%%%% Define visual properties for robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', [.9;0.9;0.9], ...
                    'EdgeAlpha', 0};
    joint_props = {'FaceColor', [0.9;0.9;0.9]};
    
    %%%%%% Joint Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for i=1:length(staubli.joint_type)
        staubli.joints(i).radius = staubli.joint_radius(i);
        staubli.joints(i).height = staubli.joint_height(i);
        staubli.joints(i).props = joint_props;
    end
    
    %%%%%% Link Defintions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    staubli.link_type = [1 0 1 0 1 1 0];
    
    staubli.links(1).radius = 0.084;
    staubli.links(1).height = 0.230;
    staubli.links(1).t0 = .115*z0;
    staubli.links(1).R0 = eye(3);
    staubli.links(1).props = link_props;
    
    staubli.links(3).radius = 0.072;
    staubli.links(3).height = 0.368;
    staubli.links(3).t0 = [.1655;0;.1125];
    staubli.links(3).R0 = eye(3);
    staubli.links(3).props = link_props;
    
    staubli.links(5).radius = 0.0585;
    staubli.links(5).height = 0.123;
    staubli.links(5).t0 = .1235*z0;
    staubli.links(5).R0 = eye(3);
    staubli.links(5).props = link_props;
    
    staubli.links(6).radius = 0.02;
    staubli.links(6).height = 0.0225;
    staubli.links(6).t0 = 0.0513*z0;
    staubli.links(6).R0 = eye(3);
    staubli.links(6).props = link_props;
    
    staubli.frame.scale = 0.1;
    staubli.frame.width = 0.01;
                
    handle = createRobot(R0, t0, staubli,'CreateFrames',cf);
end