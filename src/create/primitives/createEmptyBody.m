function handle = createEmptyBody(n)
    % CREATEEMPTYBODY
    %
    % handle = createEmptyBody()
    % handle = createEmptyBody(n) - creates n empty rigid bodies
    %
    % returns standardized format for a rigid body structure with no
    % entries
    %
    
    if nargin == 0, n = 1; end
    
    % Structure with fields in standard order
    handle = struct('bodies',cell(1,n), ...
                    'labels', cell(1,n), ...
                    'R', cell(1,n), ...
                    't', cell(1,n), ...
                    'A', cell(1,n));
    % Default values for each field (bodies is already set)
    [handle.labels] = deal({});
    [handle.R] = deal(eye(3));
    [handle.t] = deal([0;0;0]);
    [handle.A] = deal(eye(3));
                
end
    