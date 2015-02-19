function handle = updateRigidBodyAffine(A, handle)
    %
    % handle = updateRigidBodyAffine(A, handle)
    %
    % this function is intended to be a more general purpose update command
    % to bodies that are allowed to stretch / compress / shear as well as
    % rigid body transformations.  
    % It is recommended to use the UPDATERIGIDBODY for simple 
    %           rotation / translation commands.
    %
    % A is desired affine transformation of the body
    % handle is drawing structure for body undergoing transformation
    %
    % returns updated structure
    %
    % see also UPDATERIGIDBODY
    
    if isempty(handle);  return;  end
    if nargout ~= 1; error('Must have return value for handle'); end;
    
    % Determine local transformation
    A12 = A/handle.A;
    A12 = handle.R*A12*handle.R';
    t12 = (eye(3) - A12)*handle.t;
    
    for i=1:length(handle.bodies)
        V = get(handle.bodies(i),'Vertices');
        dim = size(V);
        V = V*A12' + ones(dim(1),1)*t12';
        set(handle.bodies(i),'Vertices',V);
    end
    handle.A = A;
end