function [robot, object] = removeObjectFromRobot(label, frame, robot)
    %
    % [robot, object] = removeObjectFromRobot(label, frame, robot)
    %
    % label is the label of the body to remove.  Will remove all bodies
    %   which contain the label as a substring of their label
    %       i.e. 'label1' removes 'label1', 'label1_x', 'label1_y'
    % frame is the robot frame that the object is rigid to
    % robot is a handle to the robot structure
    %
    % NOTE:  DOES NOT REMOVE BODY FROM ENVIRONMENT, SIMPLY REMOVES IT FROM
    % BEING ATTACHED TO ROBOT'S BODY FRAME
    %
    % returns:
    %       handle to updated robot structure 
    %       handle to removed object
    %
    % see also DELETEBODIES ATTACHOBJECTTOROBOT
    
    hits = [];
    
    for i = 1:length(robot.frame(frame+1).labels)
        if isempty(strfind(robot.frame(frame+1).labels{i},label))
            continue;
        end
        hits = [hits i];
    end
    
    if isempty(hits)
        return;
    end
    
    new_set = setdiff(1:length(robot.frame(frame+1).labels), hits);
    object.bodies = robot.frame(frame+1).bodies(hits);
    object.labels = robot.frame(frame+1).labels(hits);
    object.R = robot.frame(frame+1).R;
    object.t = robot.frame(frame+1).t;
    robot.frame(frame+1).bodies = robot.frame(frame+1).bodies(new_set);
    robot.frame(frame+1).labels = robot.frame(frame+1).labels(new_set);
    
end