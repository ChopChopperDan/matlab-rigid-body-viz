function combined_handle = combineRobots(handle1, handle2, name, varargin)
    %
    % combined_handle = combineRobots(handle1, handle2, name)
    % combined_handle = combineRobots(handle1, handle2, name, ...)
    %
    % serially connects the robot structure in handle2 to the end effector
    %       of the robot with id "name" in handle1.  Moves the entire base
    %       of the structure in handle2 to the tool frame.  
    %       Use 'root' as "name" to
    %       simply connect the structure handle2 to the base of the handle1
    %       structure.
    %
    % Additional optional flags:
    %       'HoldHandle2'    : Connects handle2 to tool without moving the
    %                           base frame (simply adds current offset to
    %                           robot2 kinematics)
    %                         default: 'off'
    %
    % returns a handle containing the updated rigid bodies
    %       and the combined robot structure
    %
    %  example: adding a gripper onto a single arm robot
    %       h_robot = combineRobots(h_robot, h_gripper, 'arm')
    %
    
    flags = {'HoldHandle2'};
    defaults = {'off'};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    hold_r2 = opt_values{1};
    
    % Combine associated bodies into same structure
    combined_handle.bodies = [handle1.bodies handle2.bodies];
    combined_handle.labels = [handle1.labels handle2.labels];
    % New combined handle retains coordinate system of handle1
    combined_handle.R = handle1.R;
    combined_handle.A = handle1.A;
    combined_handle.t = handle1.t;
    
    % offset for index references for bodies in master list
    nb1 = numel(handle1.bodies);
    
    % Determine where to attach structure from handle2 onto handle1
    
    if strcmpi(name, 'root')
        idx = 0;
        pOT = handle1.t;
    else
        idx = strcmpi({handle1.robots.name},name);
        if (idx == 0) 
            error('combineRobots:name_not_found', ...
                            ['Cannot find robot named ' name]); 
        end
        pOT = handle1.t + handle1.R*handle1.robots(idx).frames(end).t;
    end
    if strcmp(hold_r2,'off')
        handle2 = updateRigidBody(handle2.R, pOT, handle2);
    end
    pT2 = handle2.t - pOT;
    
    % Adjust robots in handle2
    for j=1:numel(handle2.robots)
        % Add offset to handle2 robot body indices
        handle2.robots(j).base.bodies = nb1 + ...
                handle2.robots(j).base.bodies;
        if ~isempty(handle2.robots(j).load)
            handle2.robots(j).load.bodies = nb1 + ...
                    handle2.robots(j).load.bodies;
        end
        for i=1:numel(handle2.robots(j).frames)
            handle2.robots(j).frames(i).bodies = nb1 + ...
                handle2.robots(j).frames(i).bodies;
        end
        
        % Add displacement to robot kinematics and set left field to
        %   defined robot in kinematic chain
        if strcmpi(handle2.robots(j).left,'root')
            handle2.robots(j).left = name;
            handle2.robots(j).base.R = handle2.R;
            handle2.robots(j).base.t = handle2.t;
            handle2.robots(j).kin.P(:,1) = pT2 + ... 
                handle2.robots(j).kin.P(:,1);
            
            % Add robot to right field of robot from handle1 if not
            %   attached to 'root'
            if idx > 0
                if iscellstr(handle1.robots(idx).right)
                    handle1.robots(idx).right = ...
                        [handle1.robots(idx).right handle2.robots(j).name];
                else
                    handle1.robots(idx).right = {handle2.robots(j).name};
                end
            end
        end
    end
    
    % add adjusted handles to combined handle
    combined_handle.robots = [handle1.robots handle2.robots];
    
end