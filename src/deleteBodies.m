function handle = deleteBodies(handle, identifier, varargin)
    %
    % handle = deleteBodies(handle, identifier)
    %
    % handle - drawing structure to modify
    % identifier - string to find within handle's labels and delete only
    %               the associated bodies.  To apply to all bodies
    %               within the handle, set to 'all' or []
    %
    % returns adjusted handle with deleted bodies and associated labels
    %               removed
    %
    
    if isempty(identifier) || strcmp(identifier,'all')
        hits = 1:length(handle.bodies);
    else
        hits = [];

        for i = 1:length(handle.labels)
            if isempty(strfind(handle.labels{i},identifier))
                continue;
            end
            hits = [hits i];
        end
    end
    
    if isempty(hits);  return;   end
    
    delete(handle.bodies(hits));
    
    new_set = setdiff(1:length(handle.bodies),hits);
    handle.labels = handle.labels(new_set);
    handle.bodies = handle.bodies(new_set);
end