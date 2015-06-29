function [gripper_const, gripper_structure] = ...
                                defineParallelJawGripper(param, varargin)
    % DEFINEPARALLELJAWGRIPPER
    %
    % [gripper_const, gripper_structure] = defineParallelJawGripper(param)
    %       param is struct containing fields
    %           -> aperture (maximum opening between two fingers)
    %           -> height   (height of each finger)
    %           -> fingerwidth [opt] (width of each finger)
    %           -> palm [opt] (cuboid parameterization)
    %
    % [gripper_const, gripper_structure] = ...
    %                               defineParallelJawGripper(param, ...) 
    %        allows additional optional parameters
    %
    %           'Color'     :   default [0;0;0]
    %           'Origin'    :   default [eye(3) [0;0;0]; [0 0 0] 1]
    % 
    % returns:
    %       gripper_const - struct array for the two 'robots' that make up 
    %           a generic parallel-jaw gripper with the fields
    % 
    % root
    %   -> name            : (1) 'gripper_left_jaw'
    %                        (2) 'gripper right_jaw'
    %   -> kin
    %       -> H           : [3 x 1] joint axes
    %       -> P           : [3 x 2] inter-joint translation
    %       -> joint_type  : [1 x 1] joint types
    %   -> limit
    %       -> lower_joint_limit    : lower bound for gripper opening
    %       -> upper_joint_limit    : upper bound for gripper opening
    %   -> vis
    %       -> joints      :  [1 x 1] struct array of joint definitions
    %       -> links       :  [2 x 1] struct array of link definitions
    %
    % see also CREATECOMBINEDROBOT
    
    flags = {'Color','Origin'};
    defaults = {[0;0;0], eye(4)};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    c = opt_values{1};
    origin = opt_values{2};
    
    % parse through param file
    if ~isfield(param,'fingerwidth')
        param.fingerwidth = 0.1*param.height;
    end
    
    % Extract parameters for gripper
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
    
    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    % Grab standard robot structure
    gripper_const = defineEmptyRobot(2);
    
    gripper_const(1).name = 'gripper_left_jaw';
    gripper_const(2).name = 'gripper_right_jaw';

    % define gripper as two 1dof robots
    x0 = [1;0;0]; z0 = [0;0;1];
    gripper_const(1).kin.H = R0*x0;
    gripper_const(1).kin.P = R0*[[-param.aperture/2;0;palm_param.height], ...
                            jaw_param.height/2*z0];
    gripper_const(1).kin.P(:,1) = t0 + gripper_const(1).kin.P(:,1);
    gripper_const(1).kin.joint_type = 3;
    
    gripper_const(2).kin.H = -R0*x0;
    gripper_const(2).kin.P = R0*[[param.aperture/2;0;palm_param.height], ...
                            jaw_param.height/2*z0];
    gripper_const(2).kin.P(:,1) = t0 + gripper_const(2).kin.P(:,1);
    gripper_const(2).kin.joint_type = 3;
    
    % Set kinematic limits
    gripper_const(1).limit.lower_joint_limit = 0;
    gripper_const(2).limit.lower_joint_limit = 0;
    gripper_const(1).limit.upper_joint_limit = param.aperture/2;
    gripper_const(2).limit.upper_joint_limit = param.aperture/2;

    % Visualization constants.  
    
    % fields 'joints', 'frame', and 'peripherals' are unused
    % Only the links get visualizations, 
    [gripper_const(1).vis.links, ...
     gripper_const(2).vis.links] = deal(struct('handle',cell(1,2), ...
                                            'R',cell(1,2), ...
                                            't',cell(1,2), ...
                                            'param',cell(1,2), ...
                                            'props',cell(1,2)));
    
    link_props = {'FaceColor',c,'EdgeColor',c};
    
    % left jaw + palm
    gripper_const(1).vis.links(1).handle = @createCuboid;
    gripper_const(1).vis.links(1).R = R0;
    gripper_const(1).vis.links(1).t = palm_param.height/2*R0*z0;
    gripper_const(1).vis.links(1).param = palm_param;
    gripper_const(1).vis.links(1).props = link_props;
    
    gripper_const(1).vis.links(2).handle = @createCuboid;
    gripper_const(1).vis.links(2).R = R0;
    gripper_const(1).vis.links(2).t = R0*[-jaw_param.width/2;0; ...
                                        jaw_param.height/2];
    gripper_const(1).vis.links(2).param = jaw_param;
    gripper_const(1).vis.links(2).props = link_props;
    
    % right jaw    
    gripper_const(2).vis.links(2).handle = @createCuboid;
    gripper_const(2).vis.links(2).R = R0;
    gripper_const(2).vis.links(2).t = R0*[jaw_param.width/2;0; ...
                                        jaw_param.height/2];
    gripper_const(2).vis.links(2).param = jaw_param;
    gripper_const(2).vis.links(2).props = link_props;
    
    % Define basic structure with both jaws attached to the root of the
    %   robot in parallel
    gripper_structure = defineEmptyRobotStructure(2);
    [gripper_structure.name] = gripper_const.name;
    
end