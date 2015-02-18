function robot = attachObjectToRobot(object, frame, robot)
    %
    % robot = attachObjectToRobot(object, frame, robot)
    %
    % object is a handle to a drawing structure
    % frame is the robot frame that the object is rigid to
    % robot is a handle to the robot structure
    %
    % returns handle to updated robot structure
    
    robot.frame(frame+1).bodies = [robot.frame(frame+1).bodies ...
                    object.bodies];
    robot.frame(frame+1).labels = [robot.frame(frame+1).labels ...
                    object.labels];
end