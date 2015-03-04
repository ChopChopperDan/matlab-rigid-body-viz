function [handle, object] = releaseLoad(handle, robot)
    %
    % [handle, object] = releaseLoad(handle)
    % [robot, object, robot] = releaseLoad(handle, robot)
    %
    % handle contains full information for robot structure
    % [opt] robot is nested robot handle that is carrying load
    %
    % returns:
    %       1. handle to updated full robot structure 
    %       2. handle to removed object
    %       3. [opt] handle to update nested robot structure 
    
    if nargin == 1, robot = handle.robots(1); end
    if isempty(robot.load),  object = [];   return;   end
    
    object.bodies = handle.bodies(robot.load.bodies);
    object.labels = handle.labels(robot.load.bodies);
    RT = robot.base.R*robot.frames(end).R;
    tT = robot.base.t + robot.base.R*robot.frames(end).t;    
    object.R = RT*robot.load.Rb;
    object.t = tT + RT*robot.load.tb;
    object.A = eye(3);
    robot.load = [];
    
    if nargin == 1 && nargout == 2
        handle.robots(1) = robot;
    end

end