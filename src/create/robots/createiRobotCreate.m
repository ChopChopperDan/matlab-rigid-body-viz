function handle = createiRobotCreate(R0, t0, varargin)
    %
    % handle = createiRobotCreate(R0, t0, ...)
    %
    % File to create an iRobot Create drawing object
    %
    %
    % Optional parameters for initialization are:
    %       'Color'             default: [.9;.9;.9]
    %       'CreateFrames'      default: 'off'
    %
    % depends on the following drawing package files:
    %       createCylinder.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %
    % returns handle to the iRobot Create robot structure
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0=[0;0;1]; zed = [0;0;0];
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        elseif strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('c','var'); c = [.9;0.9;0.9]; end
    if ~exist('cf','var'); cf = 'off'; end
    
    %%%%%% Define Kinematics for robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    create.H = [x0 y0 z0];
    create.P = [.015*z0 zed zed zed];
    create.joint_type = [3 3 2];
    create.name = 'create';
    
    %%%%%% Define visual properties for robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    link_props = {'FaceColor', c, 'EdgeAlpha', 1};
    
    %%%%%% Link Definitions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    create.link_type = [0 0 0 1];
    
    create.links(4).radius = .165;
    create.links(4).height = .065;
    create.links(4).t0 = [0;0;0.0325];
    create.links(4).R0 = eye(3);
    create.links(4).props = link_props;
    
    %%%%%% Define reference frame dimensions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    create.frame.scale = 0.3;
    create.frame.width = 0.015;
        
    %%%%%% Create robot objects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    handle = createRobot(R0, t0, create, 'CreateFrames', cf);
    
    %%%%%% Attach extra body parts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Make front bumper
    theta_step = pi/20;
    outer_R = 0.18;
    inner_R = 0.1;
    outer_theta = -pi/2:theta_step:pi/2;
    inner_theta = pi/3:-theta_step:-pi/3;
    
    bumper_param.polygon = [outer_R*cos(outer_theta) ...
                                inner_R*cos(inner_theta); ...
                            outer_R*sin(outer_theta) ...
                                inner_R*sin(inner_theta)];
    bumper_param.height = .065;
    bumper_props = {'FaceColor', [0.6;0.6;0.6], 'EdgeAlpha', 0};
    bumper_t0 = handle.robots.frames(end).t + R0*0.0425*z0;
    
    bumper = createPrism(R0, bumper_t0, bumper_param, bumper_props{:});
    bumper.labels = attachPrefix('bumper_',bumper.labels);
    
    % Wheels
    wheel_param.radius = .035;
    wheel_param.height = .03;
    wheel_props = {'FaceColor',[0;0;0]};
    
    leftwheel_R0 = R0*rot(x0,pi/2);
    rightwheel_R0 = R0*rot(x0,pi/2);
    leftwheel_t0 = handle.robots.frames(end).t + R0*[0;.13;.02];
    rightwheel_t0 = handle.robots.frames(end).t + R0*[0;-.13;.02];
    
    leftwheel = createCylinder(leftwheel_R0, leftwheel_t0, ...
                                    wheel_param, wheel_props{:});
    rightwheel = createCylinder(rightwheel_R0, rightwheel_t0, ...
                                    wheel_param, wheel_props{:});
    leftwheel.labels = attachPrefix('leftwheel_',leftwheel.labels);
    rightwheel.labels = attachPrefix('rightwheel_',rightwheel.labels);
    
    caster_param.radius = .01;
    caster_param.height = .02;
    caster_props = {'FaceColor',[0;0;0]};
    
    caster_R0 = R0*rot(x0,pi/2);
    caster_t0 = handle.robots.frames(end).t + R0*[.15;0;-.005];
    caster = createCylinder(caster_R0, caster_t0, ...
                                    caster_param, caster_props{:});
    caster.labels = attachPrefix('caster_',caster.labels);
    
    
    handle = attachObjectToRobot(bumper,'tool', handle);
    handle = attachObjectToRobot(leftwheel,'tool', handle);
    handle = attachObjectToRobot(rightwheel,'tool', handle);
    handle = attachObjectToRobot(caster,'tool', handle);
end
    
    