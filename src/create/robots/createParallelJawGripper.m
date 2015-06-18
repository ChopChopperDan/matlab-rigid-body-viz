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
    
    % Define parameters for gripper
    if isfield(param,'palm')
        palm_param = param.palm;
    else
        palm_param.width = param.aperture + 2*param.fingerwidth;
        palm_param.length = param.fingerwidth;
        palm_param.height = param.fingerwidth;
    end
        
    jaw_param.width = param.fingerwidth;
    jaw_param.length = param.fingerwidth;
    jaw_param.height = param.height;
    
    % define gripper as two 1dof robots
    x0 = [1;0;0]; z0 = [0;0;1];
    left_jaw_const.name = 'gripper_left_jaw';
    left_jaw_const.kin.H = R0*x0;
    left_jaw_const.kin.P = R0*[[-param.aperture/2;0;palm_param.height], ...
                            jaw_param.height/2*z0];
    left_jaw_const.kin.P(:,1) = t0 + left_jaw_const.kin.P(:,1);
    left_jaw_const.kin.joint_type = 3;
    
    right_jaw_const.name = 'gripper_right_jaw';
    right_jaw_const.kin.H = -R0*x0;
    right_jaw_const.kin.P = R0*[[param.aperture/2;0;palm_param.height], ...
                            jaw_param.height/2*z0];
    right_jaw_const.kin.P(:,1) = t0 + right_jaw_const.kin.P(:,1);
    right_jaw_const.kin.joint_type = 3;
    
    
    link_props = {'FaceColor',c,'EdgeColor',c};
    
    % left jaw + palm
    left_jaw_const.vis.links = struct('handle',cell(1,2), ...
                            'R',cell(1,2),'t',cell(1,2), ...
                            'param',cell(1,2),'props',cell(1,2));
    left_jaw_const.vis.links(1).handle = @createCuboid;
    left_jaw_const.vis.links(1).R = R0;
    left_jaw_const.vis.links(1).t = palm_param.height/2*R0*z0;
    left_jaw_const.vis.links(1).param = palm_param;
    left_jaw_const.vis.links(1).props = link_props;
    
    left_jaw_const.vis.links(2).handle = @createCuboid;
    left_jaw_const.vis.links(2).R = R0;
    left_jaw_const.vis.links(2).t = R0*[-jaw_param.width/2;0; ...
                                        jaw_param.height/2];
    left_jaw_const.vis.links(2).param = jaw_param;
    left_jaw_const.vis.links(2).props = link_props;
    
    % right jaw
    right_jaw_const.vis.links = struct('handle',cell(1,2), ...
                            'R',cell(1,2),'t',cell(1,2), ...
                            'param',cell(1,2),'props',cell(1,2));
    
    right_jaw_const.vis.links(2).handle = @createCuboid;
    right_jaw_const.vis.links(2).R = R0;
    right_jaw_const.vis.links(2).t = R0*[jaw_param.width/2;0; ...
                                        jaw_param.height/2];
    right_jaw_const.vis.links(2).param = jaw_param;
    right_jaw_const.vis.links(2).props = link_props;
    
    % Create individual robot structures
    left_jaw = createRobot(eye(3), [0;0;0], left_jaw_const, ...
                                        'CreateFrames','off');
    right_jaw = createRobot(eye(3), [0;0;0], right_jaw_const, ...
                                        'CreateFrames','off');
    % combine robots into single branched robot handle
    handle = branchRobots(left_jaw, right_jaw);    
    
end