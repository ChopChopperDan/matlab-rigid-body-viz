function robot = graspLoad(object, robot)
    %
    % robot = graspLoad(object, robot)
    %
    % object is a handle to a drawing structure for grasp.  If load already
    %   exists, will not attempt grasp
    % robot is a handle to the robot structure
    %
    % returns handle to updated robot structure with object and local
    %       transformation added to the robot.load field
    
    if ~isempty(robot.load)
        return;
    end
    
    robot.load = object;
    robot.load.Rb = robot.frame(end).R'*object.R;
    robot.load.tb = robot.frame(end).R'*(object.t - robot.frame(end).t);
end