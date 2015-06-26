function handle = updateRigidBody(R, t, handle)
    % UPDATERIGIDBODY
    %
    % handle = updateRigidBody(R, t, handle)
    %
    % R is desired orientation of the body
    % t is desired position of the body
    % handle is drawing structure for body undergoing transformation
    %
    % returns updated structure
    
    % Make sure the updated handle is returned.
    if isempty(handle);  return;  end
    if nargout ~= 1 
        error('updateRigidBody:no_return_value', ...
                            'Must have return value for handle'); 
    end
    
    % Determine local transformation
    R12 = R*handle.R';
    t12 = t - R12*handle.t;
    
    for i=1:length(handle.bodies)
        V = get(handle.bodies(i),'Vertices');
        V = V*R12';
        V(:,1) = V(:,1) + t12(1);
        V(:,2) = V(:,2) + t12(2);
        V(:,3) = V(:,3) + t12(3);
        set(handle.bodies(i),'Vertices',V);
    end
    handle.R = R;
    handle.t = t;
end