function [kin, chain] = get_kinematic_chain(const, structure, name_base, name_end)
    % GET_KINEMATIC_CHAIN
    % 
    % kin = get_kinematic_chain(const, structure, name_base, name_end)
    % [kin, chain] = get_kinematic_chain(const, structure, name_base, name_end)
    %
    % computes the kinematic chain between the robots in handle that have
    % names 'name_base' and 'name_end'.
    %
    % returns kinematic structure with all terms connecting the two robots
    %   kin
    %       -> H          : [h_1 h_2 ... h_n]  
    %                       (3 x n) actuator axes
    %       -> P          : [p_{O,1} p_{1,2} ... p_{n,T}] 
    %                       (3 x n + 1) actuator displacements
    %       -> joint_type : n-vector of joint types
    %                       0 - rotational
    %                       1 - prismatic
    %                       2 - mobile rotation
    %                       3 - mobile translation
    %
    % chain returns a cell string containing the names of all robots in the
    % kinematic path between name_base and name_end
        
    if all(strcmpi({const.name}, name_base) == 0) || ...
            all(strcmpi({const.name}, name_end) == 0)
        error('get_kinematic_chain:name_not_found', ...
                'No robot constants found for provided robot names');
    end
    
    if all(strcmpi({structure.name}, name_base) == 0) || ...
            all(strcmpi({structure.name}, name_end) == 0)
        error('get_kinematic_chain:name_not_found', ...
                'No robot structure found for provided robot names');
    end
    
    kin = struct('H',[],'P',[0;0;0],'joint_type',[]);
    chain = {};
    
    name = name_end;
    
    % explore backwards from 'name_end' to find entire chain
    for n=1:numel(const)
        idx = strcmpi({const.name}, name);
        kin_left = const(idx).kin;
                
        kin.H = [kin_left.H kin.H];
        kin.P(:,1) = kin_left.P(:,end) + kin.P(:,1);
        kin.P = [kin_left.P(:,1:end-1) kin.P];
        kin.joint_type = [kin_left.joint_type kin.joint_type];
        
        chain = [name chain];
        
        name = structure(idx).left;
        
        if strcmpi(name, 'root')
            break;
        end
    end
    
    if all(strcmpi(chain, name_base) == 0)
        error('get_kinematic_chain:robots_not_connected', ...
           ['Could not find chain connecting ' name_base ' to ' name_end]);
    end
end