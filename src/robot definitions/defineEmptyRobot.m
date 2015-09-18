function empty_robot_const = defineEmptyRobot(n)
    % DEFINEEMPTYROBOT
    %
    % empty_robot_const = defineEmptyRobot()
    % empty_robot_const = defineEmptyRobot(n)
    %
    % Defines a generic robot constant file that is consistent with the
    % matlab-rigid-body-viz toolbox.  Will define 'n' robots with the
    % standard structure for a robot needed by createRobot
    %
    % see also CREATEROBOT DEFINEEMPTYROBOTSTRUCTURE
    
    if nargin == 0, n = 1;  end
    if n == 0, empty_robot_const = []; return; end
    
    % Base structure needs fields for name, kinematics, actuator limits, 
    %   and visualization constants.
    empty_robot_const = struct('name',    cell(1,n), ...
                               'kin',     cell(1,n), ...
                               'limit',   cell(1,n), ...
                               'vis',     cell(1,n));
    
    % Standard kinematics fields
    [empty_robot_const.kin] = deal(struct('H',          [], ...
                                          'P',          [], ...
                                          'joint_type', []));
    
    % Standard actuator limits
    [empty_robot_const.limit] = deal(struct('lower_joint_limit',  [], ...
                                            'upper_joint_limit',  [], ...
                                            'velocity_limit',     [], ...
                                            'effort_limit',       []));
    
    % Standard visualization fields
    [empty_robot_const.vis] = deal(struct('joints',         [], ...
                                          'links',          [], ...
                                          'frame',          [], ...
                                          'peripherals',    []));
    
    
end