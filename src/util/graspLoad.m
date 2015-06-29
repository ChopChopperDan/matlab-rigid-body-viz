function handle = graspLoad(object, handle, name)
    %
    % handle = graspLoad(object, handle, name)
    %
    % object is a handle to a drawing structure for grasp.  If load already
    %   exists, will not attempt grasp
    % handle contains robot structure
    % name is the string id of the robot that is grasping the load
    %
    % returns handle to updated robot structure with object and local
    %       transformation added to the robots.load field for the named
    %       robot
    
    
    idx = strcmpi({handle.robots.name}, name);
    if all(idx == 0) 
        error('graspLoad:robot_not_found', ...
                'name passed to graspLoad not found in robot structure');
    end
    if ~isempty(handle.robots(idx).load),  return;   end

    
    nb = numel(handle.bodies);
    Rb = handle.robots(idx).base.R;
    tb = handle.robots(idx).base.t;
    RT = Rb * handle.robots(idx).frames(end).R;
    tT = tb + Rb*handle.robots(idx).frames(end).t;
    
    handle.robots(idx).load = struct(...
                            'bodies', nb + (1:numel(object.bodies)), ...
                            'R', Rb'*object.R, ...
                            't', Rb'*(object.t - tb), ...
                            'A', object.A, ...
                            'Rb', RT'*object.R, ...
                            'tb', RT'*(object.t - tT));
    
    handle.bodies = [handle.bodies object.bodies];
    handle.labels = [handle.labels object.labels];
end