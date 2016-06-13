function robot_const = addVisPeripheral(robot_const, name, id, frame, handle, R, t, param, props)
    % ADDVISPERIPHERAL
    % 
    % robot_const = addVisPeripheral(robot_const, name, id, frame, handle, R, t, param, props)
    
    idx = strcmpi(name, {robot_const.name});
    if ~any(idx), return; end
    
    periph_struct = struct('id', id, ...
                            'frame', frame, ...
                            'handle', handle, ...
                            'R', R, ...
                            't', t, ...
                            'param', param, ...
                            'props', {props});
    
    if isempty(robot_const(idx).vis.peripherals)
        robot_const(idx).vis.peripherals = periph_struct;
                                
    else
        n = numel(robot_const(idx).vis.peripherals);
        robot_const(idx).vis.peripherals(n + 1) = periph_struct;
    end
end
