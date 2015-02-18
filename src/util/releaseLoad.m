function [robot, object] = releaseLoad(robot)
    %
    % [robot, object] = releaseLoad(robot)
    %
    % robot is a handle to the robot structure
    %
    % NOTE:  DOES NOT REMOVE BODY FROM ENVIRONMENT, SIMPLY REMOVES IT FROM
    % BEING ATTACHED TO ROBOT'S END EFFECTOR FRAME
    %
    % returns:
    %       handle to updated robot structure 
    %       handle to removed object
    
    if isempty(robot.load)
        object = [];
        return;
    end
    
    object.bodies = robot.load.bodies;
    object.labels = robot.load.labels;
    object.R = robot.frame(end).R*robot.load.Rb;
    object.t = robot.frame(end).t + robot.frame(end).R*robot.load.tb;
    robot.load = [];
    
end