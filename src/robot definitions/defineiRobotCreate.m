function create_const = defineiRobotCreate(origin, varargin)
    %
    % create_const = defineiRobotCreate()
    % create_const = defineiRobotCreate(origin) - origin is [4 x 4] matrix
    % denoting orientation and translation of Staubli tx40 with respect 
    % to the world frame
    % create_const = defineiRobotCreate(origin, 'Color', color) - default
    % color is [0.9 0.9 0.9]
    %
    % define-file for the iRobot Create mobile robot.  Returns struct with
    % the following form:
    %
    % root
    %   -> name         : string denoting the name of the robot
    %   -> kin          : container for kinematics
    %       -> H                 : [3 x 3] joint axes
    %       -> P                 : [3 x 4] inter-joint displacements
    %       -> joint_type        : [1 x 3] joint types
    %   -> vis          : container for visualization
    %       -> joints            :  3 index struct array containing fields
    %                               param
    %                               props
    %       -> links             :  4 index cell array containing fields
    %                               handle
    %                               R
    %                               t
    %                               param
    %                               props
    %       -> frame             : struct containing appropriate sized
    %                               dimensions for visualizing the 
    %                               3D frames
    %       -> peripherals       : struct containing extra visual
    %                               attachments to the robot
    %
    % see also CREATEROBOT
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('c','var'); c = [.9;0.9;0.9]; end
    if nargin == 0, origin = [eye(3) zed; zed' 1];  end
    
    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    % Name
    create_const.name = 'create';
    
    % Kinematic constants
    create_const.kin.H = R0*[x0 y0 z0];
    create_const.kin.P = R0*[.015*z0 zed zed zed];
    create_const.kin.P(:,1) = t0 + create_const.kin.P(:,1);
    create_const.kin.joint_type = [3 3 2];
    
    % Visualization constants
    
    % no joint visualizations
    create_const.vis.joints = struct('param',cell(1,3),'props',cell(1,3));
    
    % only 'link' is at tool frame
    create_const.vis.links = struct('handle',cell(1,4), ...
                            'R',cell(1,4),'t',cell(1,4), ...
                            'param',cell(1,4),'props',cell(1,4));
    
    create_const.vis.links(4).handle = @createCylinder;
    create_const.vis.links(4).R = eye(3);
    create_const.vis.links(4).t = [0;0;0.0325];
    create_const.vis.links(4).param = struct('radius',.165,'height',.065);
    create_const.vis.links(4).props = {'FaceColor', c, 'EdgeAlpha', 1};
    
    % appropriate scale for coordinate frame
    create_const.vis.frame = struct('scale', 0.3, 'width', 0.015);
    
    % peripherals to look more like actual iRobot create 
    create_const.vis.peripherals = struct('frame', cell(1,4), ...
                            'id', cell(1,4), 'handle',cell(1,4), ...
                            'R',cell(1,4),'t',cell(1,4), ...
                            'param',cell(1,4),'props',cell(1,4));

    % Front bumper
    bumper_props = {'FaceColor', [0.6;0.6;0.6], 'EdgeAlpha', 0};
    theta_step = pi/20;    outer_R = 0.18;    inner_R = 0.1;
    outer_theta = -pi/2:theta_step:pi/2;
    inner_theta = pi/3:-theta_step:-pi/3;
    
    bumper_param.height = .065;
    bumper_param.polygon = [outer_R*cos(outer_theta) ...
                                inner_R*cos(inner_theta); ...
                            outer_R*sin(outer_theta) ...
                                inner_R*sin(inner_theta)];
    
    create_const.vis.peripherals(1).id = 'bumper';
    create_const.vis.peripherals(1).frame = 'tool';
    create_const.vis.peripherals(1).handle = @createPrism;
    create_const.vis.peripherals(1).R = R0;
    create_const.vis.peripherals(1).t = 0.0425*R0*z0;
    create_const.vis.peripherals(1).param = bumper_param;
    create_const.vis.peripherals(1).props = bumper_props;
    
    % Wheels / caster
    create_const.vis.peripherals(2).id = 'left_wheel';
    create_const.vis.peripherals(2).frame = 'tool';
    create_const.vis.peripherals(2).handle = @createCylinder;
    create_const.vis.peripherals(2).R = R0*rot(x0,pi/2);
    create_const.vis.peripherals(2).t = R0*[0;.13;.02];
    create_const.vis.peripherals(2).param = struct('radius',0.035, ...
                                                    'height',0.03);
    create_const.vis.peripherals(2).props = {'FaceColor',[0;0;0]};
    
    create_const.vis.peripherals(3).id = 'right_wheel';
    create_const.vis.peripherals(3).frame = 'tool';
    create_const.vis.peripherals(3).handle = @createCylinder;
    create_const.vis.peripherals(3).R = R0*rot(x0,pi/2);
    create_const.vis.peripherals(3).t = R0*[0;-.13;.02];
    create_const.vis.peripherals(3).param = struct('radius',0.035, ...
                                                    'height',0.03);
    create_const.vis.peripherals(3).props = {'FaceColor',[0;0;0]};
    
    create_const.vis.peripherals(4).id = 'caster';
    create_const.vis.peripherals(4).frame = 'tool';
    create_const.vis.peripherals(4).handle = @createCylinder;
    create_const.vis.peripherals(4).R = R0*rot(x0,pi/2);
    create_const.vis.peripherals(4).t = R0*[.15;0;-.005];
    create_const.vis.peripherals(4).param = struct('radius',0.01, ...
                                                    'height',0.02);
    create_const.vis.peripherals(4).props = {'FaceColor',[0;0;0]};
    
    