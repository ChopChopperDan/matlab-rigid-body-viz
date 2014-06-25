function handle = setProperty(handle, identifier, varargin)
    %
    % handle = setProperty(handle, identifier, ...)
    %
    % Sets property for passed handle's bodies that have the identifier
    %   string in their label
    % handle - drawing structure to modify
    % identifier - string to find within handle's labels and modify only
    %               the associated bodies.  To apply to all bodies
    %               associated with the handle, set to 'all' or []
    %
    % Possible properties to modify are:
    %           'FaceColor'
    %           'FaceAlpha'
    %           'LineWidth'
    %           'EdgeColor'
    %           'EdgeAlpha'
    %
    % returns adjusted handle to drawing structure
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
    
    set(handle.bodies(hits),varargin{:});
end