function handle = createCombinedRobot(robots, structure)
    % CREATECOMBINEDROBOT
    %
    % handle = createCombinedRobot(robots, structure)
    %
    % robots : array of structures containing robot definitions as required
    %           by CREATEROBOT
    % structure : structure array containing information for each defined 
    %           robot that establishes how to connect the passed robots 
    %           into one single robot handle.
    %           Each struct in the array must correspond to an entry in the
    %           robots parameter, and have the form
    %   root
    %       -> name                 : name of the robot defined in 'robots'
    %       -> left                 : name for the robot that is connected 
    %                                   to the base of this robot
    %       -> right                : cell string array containing the 
    %                                   names of all robots who attach 
    %                                   to the end of this robot
    %       -> create_properties    : cell containing additional 
    %                                   parameters to send to CREATEROBOT
    %       -> combine_properties   : cell containing additional 
    %                                   parameters to send to COMBINEROBOTS
    %       
    %
    % returns handle to final robot containing all robots in the
    %       appropriate structure
    %
    % see also CREATEROBOT COMBINEROBOTS
    
    n_r = numel(robots);
    
    robot_handles = createEmptyBody(n_r);
    [robot_handles.robots] = deal([]);
    
    % create all passed robots
    for i=1:n_r
        s_idx = strcmpi({structure.name}, robots(i).name);
        robot_handles(i) = createRobot(robots(i), ...
                            structure(s_idx).create_properties{:});
    end

    % combine handles into chains where each distinct structure 
    %   is connected to the root
    robot_chains = createEmptyBody(n_r);
    [robot_chains.robots] = deal([]);
    
    
    n_c = 0;
    for i=1:n_r
        s_idx = strcmpi({structure.name}, robots(i).name);
        if strcmpi(structure(s_idx).left,'root')
            n_c = n_c + 1;
            % Seed this chain with a robot attached to the root
            if isempty(structure(s_idx).right)
                robot_chains(n_c) = robot_handles(i);
            else
                robot_chains(n_c) = createRobotChain(robot_handles(i), ...
                            robot_handles, structure, robots(i).name);
            end
        end
    end
        
    % combine all robot chains into single final robot
    if n_c == 1
        handle = robot_chains(1);
    else
        handle = combineRobots(robot_chains(1), robot_chains(2), 'root');
        if n_c > 2
            for i=3:n_c
                handle = combineRobots(handle, robot_chains(i), 'root');
            end
        end
    end
    
end

function handle = createRobotChain(handle, robot_handles, structure, name)
    % Walk through structure to attach all robots together that are connect
    %   through this chain
    
    s_idx = strcmpi({structure.name},name);
    robot_handle_robots = [robot_handles.robots];
    
    for i=1:numel(structure(s_idx).right)
        r_idx = strcmpi({robot_handle_robots.name}, ...
                            structure(s_idx).right{i});
        s_r_idx = strcmpi({structure.name}, structure(s_idx).right{i});
        handle = combineRobots(handle, robot_handles(r_idx), name, ...
                                structure(s_r_idx).combine_properties{:});
        handle = createRobotChain(handle, robot_handles, structure, ...
                                structure(s_idx).right{i});
        
    end
    
end