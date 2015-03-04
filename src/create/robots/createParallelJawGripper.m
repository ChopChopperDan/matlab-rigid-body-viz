function handle = createParallelJawGripper(R0, t0, param, varargin)
    % 
    % handle = createParallelJawGripper(R0, t0, param, ...)
    %
    % R0 is orientation of the gripper 
    % t0 is the bottom of the base of the gripper 
    % param is struct containing fields
    %       aperture (maximum opening between two fingers)
    %       height   (height of each finger)
    %       [opt] fingerwidth (width of each finger)
    %       [opt] palm (cuboid parameterization)
    % 
    % Additional parameters include:
    %       'Color':   default: [0;0;0]
    %
    % depends on the following drawing package files:
    %       createCuboid.m
    %       attachPrefix.m
    %
    % returns handle to drawing structure
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0;0;0]; end
    if ~isfield(param,'fingerwidth')
        param.fingerwidth = 0.1*param.height;
    end
    if isfield(param,'palm')
        palm_param = param.palm;
    else
        palm_param.width = param.aperture + 2*param.fingerwidth;
        palm_param.length = param.fingerwidth;
        palm_param.height = param.fingerwidth;
    end
        
    finger_param.width = param.fingerwidth;
    finger_param.length = param.fingerwidth;
    finger_param.height = param.height;
    
    % define gripper as two 1dof robots
    x0 = [1;0;0]; z0 = [0;0;1]; zed = [0;0;0];
    left_finger.H = x0;
    left_finger.P = [palm_param.height*z0 - param.aperture/2*x0, zed];
    left_finger.joint_type = 3;
    left_finger.name = 'gripper_left_jaw';
    
    right_finger.H = -x0;
    right_finger.P = [palm_param.height*z0 + param.aperture/2*x0, zed];
    right_finger.joint_type = 3;
    right_finger.name = 'gripper_right_jaw';
    
    link_props = {'FaceColor',c,'EdgeColor',c};
    
    % Define left jaw (as -x)
    left_finger.link_type = [0 2];
    left_finger.links(2) = finger_param;
    left_finger.links(2).R0 = eye(3);
    left_finger.links(2).t0 = -finger_param.width/2*x0 + ...
                                finger_param.height/2*z0;
    left_finger.links(2).props = link_props;
    
    % Define right jaw (as +x)
    right_finger.link_type = [0 2];
    right_finger.links(2) = finger_param;
    right_finger.links(2).R0 = eye(3);
    right_finger.links(2).t0 = finger_param.width/2*x0 + ...
                                finger_param.height/2*z0;
    right_finger.links(2).props = link_props;
    
    % Create individual robot structures
    left_jaw = createRobot(R0, t0, left_finger, ...
                                        'CreateFrames','off');
    right_jaw = createRobot(R0, t0, right_finger, ...
                                        'CreateFrames','off');
    % attach palm to base of one of the robots
    palm = createCuboid(R0, t0 + R0*palm_param.height/2*z0, ...
                                        palm_param, link_props{:});
    palm.labels = attachPrefix('palm_', palm.labels);
    left_jaw = attachObjectToRobot(palm,'base',left_jaw);
    % combine robots into single branched robot handle
    handle = branchRobots(left_jaw, right_jaw);    
    
end