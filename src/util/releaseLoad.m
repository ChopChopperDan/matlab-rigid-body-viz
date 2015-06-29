function [handle, object] = releaseLoad(handle, name)
    %
    % [handle, object] = releaseLoad(handle, name)
    %
    % handle to robot structure currently carrying load
    % name is string id to robot carrying load
    %
    % returns:
    %       handle to updated full robot structure 
    %       object as released load
    
    idx = strcmpi({handle.robots.name}, name);
    if all(idx == 0) 
        error('releaseLoad:robot_not_found', ...
                'name passed to releaseLoad not found in robot structure');
    end
    if isempty(handle.robots(idx).load),  return;   end
    
    Rb = handle.robots(idx).base.R;
    tb = handle.robots(idx).base.t;
    RT = Rb * handle.robots(idx).frames(end).R;
    tT = tb + Rb*handle.robots(idx).frames(end).t;
    
    load = handle.robots(idx).load;
    
    % Use standard rigid body definition to maintain consistency
    object = createEmptyBody();
    object.bodies = handle.bodies(load.bodies);
    object.labels = handle.labels(load.bodies);
    object.R = RT*load.Rb;
    object.t = tT + RT*load.tb;
    object.A = load.A;
    
    % Remove bodies from handle
    nb = numel(handle.bodies);
    handle.bodies = handle.bodies(setdiff(1:nb, load.bodies));
    handle.labels = handle.labels(setdiff(1:nb, load.bodies));
    
    % Clear load field in robot
    handle.robots(idx).load = [];
    
    
end