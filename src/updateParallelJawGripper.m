function handle = updateParallelJawGripper(aperture,handle)
    %
    % handle = updateParallelJawGripper(aperture, handle)
    %
    % aperture is the desired opening between the two grippers 
    %   (0 is completely closed)
    % handle is drawing structure for ParallelJawGripper object
    %
    % returns updated structure
    %
    % see also CREATEPARALLELJAWGRIPPER
    
    if (aperture < 0)
        disp('Aperture must be positive');
        aperture = 0;
    elseif (aperture > handle.gripper_info.param.aperture)
        disp('Cannot exceed maximum aperture width');
        aperture = handle.gripper_info.param.aperture;
    end
    
    il = [];
    ir = [];
    for i = 1:length(handle.labels)
        if ~isempty(strfind(handle.labels{i},'ljaw'))
            il = [il i];
        elseif ~isempty(strfind(handle.labels{i},'rjaw'))
            ir = [ir i];
        end
    end
    
    if isempty(il) || isempty(ir)
        disp('Must be Parallel Jaw Structure');
        return;
    end
    
    ljaw.bodies = handle.bodies(il);
    ljaw.labels = handle.labels(il);
    ljaw.R = handle.R;
    ljaw.t = handle.t;
    
    rjaw.bodies = handle.bodies(ir);
    rjaw.labels = handle.labels(ir);
    rjaw.R = handle.R;
    rjaw.t = handle.t;
    
    R = handle.R*handle.gripper_info.RT;
    dx = (handle.gripper_info.aperture - aperture)/2;
    
    tl = handle.t + R*[dx;0;0];
    tr = handle.t - R*[dx;0;0];
    
    ljaw = updateRigidBody(ljaw.R, tl, ljaw);
    rjaw = updateRigidBody(rjaw.R, tr, rjaw);
    
    handle.bodies(il) = ljaw.bodies;
    handle.bodies(ir) = rjaw.bodies;
    handle.gripper_info.aperture = aperture;
end