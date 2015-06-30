function theta = get_angle_structure(handle)
    % GET_ANGLE_STRUCTURE
    %
    % theta = get_angle_structure(handle)
    %
    % generates struct array to set the angles for the passed robot handle
    % with the form 
    % root
    %       -> name         : name of robot
    %       -> state        : vector with the same size as the number of
    %                           actuators for the associated robot
    %
    % array is the same size as the number of robots in handle
    
    n = numel(handle.robots);
    
    theta = struct('name',cell(1,n), 'state', cell(1,n));
    for i=1:n
        theta(i).name = handle.robots(i).name;
        theta(i).state = zeros(1,numel(handle.robots(i).kin.joint_type));
    end
end