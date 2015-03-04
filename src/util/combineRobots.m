function handle = combineRobots(handle1, handle2)
    %
    % handle = combineRobots(handle1, handle2)
    %
    % serially connects the kinematics of robot1 and robot2 by nesting
    %       robot2 in the .robots field of the first robot associated with 
    %       handle1
    % returns a handle containing the updated rigid bodies
    %       and the combined robot structure
    %
    %  example: adding a gripper onto a single arm robot
    %       h_robot = combineRobots(h_robot, h_gripper)
    %
    %   NOTE: handle1 should only contain a single serial chain robot to 
    %       avoid overwriting robot structures
    %
    % see also BRANCHROBOTS
    
    robot1 = handle1.robots(1);
    robot2 = handle2.robots;
    
    handle = handle1;
    
    nb = numel(handle.bodies);
    
    for i=1:numel(robot2)
        % Before we can combine robots, need to update robot2 to have index
        % offset and update base frames of each robot to adjust by the end
        % frame of robot1
        robot2(i) = recursiveRobotAdjust(robot2(i), nb, ...
                                        robot1.frames(end).R, ...
                                        robot1.frames(end).t);
    end
   
    % combine bodies vectors
    handle.bodies = [handle.bodies handle2.bodies];
    handle.labels = [handle.labels handle2.labels];
    handle.robots(1).robots = robot2;
end

function robot = recursiveRobotAdjust(robot, index_offset, R,t)
    
    % Add index offset to all robot body indices, adjust robot base frames
    % by R, t
    
    robot.base.bodies = index_offset + robot.base.bodies;
    for i=1:numel(robot.frames)
        robot.frames(i).bodies = index_offset + robot.frames(i).bodies;
    end
    robot.base.R = R*robot.base.R;
    robot.base.t = t + R*robot.base.t;
    
    % Check whether the handle is a leaf, in which case we can return, or
    % whether we need to step down to a lower branch
    if ~isfield(robot,'robots') || isempty(robot.robots)
        % found a leaf, can return
        return;
    else
        % pass information to lower branch and update the transformation
        % terms of the lower branch to reflect the local transformation
        % from the base frame to the end of this serial chain
        for i=1:numel(robot.robots)
            robot.robots(i) = recursiveRobotAdjust(robot.robots(i), ...
                                                    index_offset, ...
                                                    R, t);
        end
    end
end