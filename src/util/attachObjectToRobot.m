function [handle, robot] = attachObjectToRobot(object, frame, handle, robot)
    %
    % handle = attachObjectToRobot(object, frame, handle)
    % [handle, robot] = attachObjectToRobot(object, frame, handle, robot)
    %
    % object is a handle to a drawing structure
    % frame is the robot frame that the object is rigid to
    %       can be     [1 - n] to be attaached to the frame associated 
    %                           with a particular joint, 
    %                  'base' to be attached to the robot's base (0) frame
    %                  'tool' to be attached to the robot's tool (T) frame
    % handle is structure containing robot information
    % [opt] robot is structure containing specific nested robot
    %       if not specified, simply takes first robot index from handle
    %
    % returns updated handle to robot structure
    %         [opt] robot structure
    
    if nargin == 3,  robot = handle.robots(1);  end
    
    % get a count of the current number of bodies associated with handle
    nb = numel(handle.bodies);
    
    
    % add body handles from the object to the handle
    handle.bodies = [handle.bodies object.bodies];
    handle.labels = [handle.labels ...
                attachPrefix([robot.name '_' num2str(frame) '_'], ...
                                object.labels)];
    
    % add references to the body labels to the desired frame
    if isnumeric(frame) && frame > 1 && frame < length(robot.kin.type)
        robot.frames(frame).bodies = [robot.frames(frame).bodies ...
                                    nb + (1:numel(object.bodies))];
        
    elseif strcmp(frame,'tool')
        robot.frames(end).bodies = [robot.frames(end).bodies ...
                                    nb + (1:numel(object.bodies))];
        
    elseif strcmp(frame,'base')
        robot.base.bodies = [robot.base.bodies ...
                                    nb + (1:numel(object.bodies))];
        
    else
        disp('Unrecognized frame label. Not attached');
        return;
    end
    
    if nargin == 3
        handle.robots(1) = robot;
    end
    
end