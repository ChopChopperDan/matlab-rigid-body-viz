function argout = mrbv_parse_input(argin, flags, defaults)
    % MRBV_PARSE_INPUT
    %
    % argout = mrbv_parse_input(argin, flags, defaults)
    %
    % given a cell array 'argin', tries to find any of the strings in the
    % cell array 'flags'.  Assumes argin is the form 
    %                       {flag, value, flag, value}
    %
    % 'argout' is a cell array the same size as 'flags'.  Each value
    %       matches the passed value within 'argin' or sets itself to the
    %       provided value in 'defaults'
    
    % extract argin into flags and values.
    if mod(numel(argin),2) ~= 0
        error('mrbv_parse:incorrect_input', ...
                'argin should have form {flag, value, flag, value}');
    end
    
    argout = defaults;
    
    in_flags = argin(1:2:end);
    in_values = argin(2:2:end);
    
    for i=1:numel(in_flags)
        flag_match = strcmp(in_flags{i},flags);
        if any(flag_match)
            argout(flag_match) = in_values(i);
        end
    end
    
end