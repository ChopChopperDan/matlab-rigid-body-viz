function empty_robot_structure = defineEmptyRobotStructure(n)
    % DEFINEEMPTYROBOTSTRUCTURE
    %
    % empty_robot_structure = defineEmptyRobotStructure()
    % empty_robot_structure = defineEmptyRobotStructure(n)
    %
    % Defines a generic structure constants file that is consistent with
    % the matlab-rigid-body-viz toolbox.  Will define 'n' robots that are
    % all attached to the root.
    %
    % see also CREATECOMBINEDROBOT DEFINEEMPTYROBOT
    
    if nargin == 0, n = 1;  end
    
    empty_robot_structure = struct('name', cell(1,n), ...
                                   'left', cell(1,n), ...
                                   'right', cell(1,n), ...
                                   'create_properties', cell(1,n), ...
                                   'combine_properties', cell(1,n));
    [empty_robot_structure.left] = deal('root');
    [empty_robot_structure.right] = deal({});
    [empty_robot_structure.create_properties] = deal({});
    [empty_robot_structure.combine_properties] = deal({});
end