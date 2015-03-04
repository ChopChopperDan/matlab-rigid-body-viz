function handle = branchRobots(handle1, handle2)
    %
    % handle = branchRobots(handle1, handle2)
    %
    % combines handle1, handle2, and other robot handles into a single
    %   branched robot structure 
    % handle contains all robots in the .robots field indexed in the order
    %   they are sent in the parameters
    %
    % handle will retain the same base parameters (R, t) as handle1, and
    %   the rest of the robots are parameterized with respect to this
    %   coordinate frame
    %
    %   example: connecting two arms into a multiarmed robot
    %       h_multiarm = branchRobots(h_left_arm, h_right_arm)
    %
    % see also COMBINEROBOTS
    handle = handle1;
    nb = numel(handle.bodies);
    
    for i=1:numel(handle2.robots)
        
        % Before we can combine robots, need to update robot2 to have index
        % offset
        handle2.robots(i) = recursiveRobotAdjust(handle2.robots(i), nb);
    end
    
    % combine bodies vectors
    handle.bodies = [handle.bodies handle2.bodies];
    handle.labels = [handle.labels handle2.labels];
    nr1 = numel(handle.robots); nr2 = numel(handle2.robots);
    handle.robots(nr1+(1:nr2)) = handle2.robots;
end

function robot = recursiveRobotAdjust(robot, index_offset)
    
    % Add index offset to all robot body indices
    
    robot.base.bodies = index_offset + robot.base.bodies;
    for i=1:numel(robot.frames)
        robot.frames(i).bodies = index_offset + robot.frames(i).bodies;
    end
    
    % Check whether the handle is a leaf, in which case we can return, or
    % whether we need to step down to a lower branch
    if ~isfield(robot,'robots') || isempty(robot.robots)
        % found a leaf, can return
        return;
    else
        % pass information to lower branch
        for i=1:numel(robot.robots)
            robot.robots(i) = recursiveRobotAdjust(robot.robots(i), ...
                                                    index_offset);
        end
    end
end