function handle = attachObjectToRobot(object, frame, handle, name)
    %
    % handle = attachObjectToRobot(object, frame, handle, robot)
    %
    % object is a handle to a drawing structure
    % frame is the robot frame that the object is rigid to
    %       can be     [1 - n] to be attached to the frame associated 
    %                           with a particular joint, 
    %                  'base' to be attached to the robot's base (0) frame
    %                  'tool' to be attached to the robot's tool (T) frame
    % handle is structure containing robot information
    % name is string denoting robot id
    %
    % returns updated handle to robot structure
    
    idx = strcmpi({handle.robots.name},name);
    if idx == 0 
        error('attachObjectToRobot:name_not_found', ...
                        ['No robot with name ' name]);
    end
    
    % get a count of the current number of bodies associated with handle
    nb = numel(handle.bodies);
    % add body handles from the object to the handle
    handle.bodies = [handle.bodies object.bodies];
    handle.labels = [handle.labels ...
            attachPrefix([name '_' num2str(frame) '_'], object.labels)];
    
    % add references to the body labels to the desired frame
    if isnumeric(frame) && frame > 1 && frame < length(name.kin.type)
        handle.robots(idx).frames(frame).bodies = ...
                        [handle.robots(idx).frames(frame).bodies ...
                                    nb + (1:numel(object.bodies))];
        
    elseif strcmp(frame,'tool')
        handle.robots(idx).frames(end).bodies = ...
                        [handle.robots(idx).frames(end).bodies ...
                                    nb + (1:numel(object.bodies))];
        
    elseif strcmp(frame,'base')
        handle.robots(idx).base.bodies = ...
                        [handle.robots(idx).base.bodies ...
                                    nb + (1:numel(object.bodies))];
        
    else
        error('attachObjectToRobot:frame_not_found', ...
                        'Frame label not recognized');
    end
    
    
end