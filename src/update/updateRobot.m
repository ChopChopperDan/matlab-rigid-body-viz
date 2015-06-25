function handle = updateRobot(theta, handle)
    %
    % handle = updateRobot(theta, handle)
    %
    % theta is a structure containing the actuator displacements for all
    %       robots that need updating.  The fields must have the form
    %   root
    %       -> names    : cell string array containing each robot to be
    %                       updated **Does not need to be every robot
    %                       defined with the robot structure**
    %                       {'robot 1', 'robot 2', ... , 'robot n'}
    %       -> states   : cell array containing the displacements for each
    %                       robot named in the 'names' field.
    %                       {theta_1, theta_2, ... , theta_n}
    %                       
    % handle is the robot drawing structure that needs updating.  Returns
    %       the updated structure.
    %
    % depends on
    %           updateRigidBody.m
    %
    % see also UPDATERIGIDBODY CREATEROBOT
    
    % check correctness of theta input
    if ~isfield(theta,'names') || ~isfield(theta,'states')
        error('updateRobot:incorrect_struct', ...
                    'theta must have fields "names" and "states"');  
    end
    if ~iscellstr(theta.names) || ~iscell(theta.states)
        error('updateRobot:incorrect_field_types', ...
                    '"names", "states" must be cell arrays');
    end
    if numel(theta.names) ~= numel(theta.states)
        error('updateRobot:incorrect_theta_dimensions', ...
                'theta fields "names" and "states" must be equal size');
    end
    
    for i=1:numel(handle.robots)
        handle = updateSingleRobot(theta, handle, handle.robots(i).name);
    end
    
        
end

function handle = updateSingleRobot(theta, handle, name)
    % Calculate forward kinematics for this serial chain
    
    % Locate robot index via passed name parameter
    idx = strcmpi({handle.robots.name},name);
    robot = handle.robots(idx);
    
    % Extract relevant information from theta
    theta_idx = strcmpi(theta.names,name);
    if theta_idx == 0 
        % If name isn't found in theta to update, use same saved angles
        q = robot.kin.state;
    else
        q = theta.states{idx};
    end
    
    % Need base structure coordinate frame
    R0 = handle.R;
    t0 = handle.t;
    
    % Look at left handle to see lower kinematic chain and current position
    % for the base of the robot
    if strcmpi(robot.left,'root')
        Ri = eye(3);
        ti = [0;0;0];
    else
        left_idx = strcmpi({handle.robots.name},robot.left);
        Ri = handle.robots(left_idx).frames(end).R;
        ti = handle.robots(left_idx).frames(end).t;
    end
    
    % create dummy rigidbody struct for updateRigidBody
    frame = struct('bodies',[],'R',eye(3),'t',[0;0;0]);
    
    % update base frame
    frame.bodies = handle.bodies(robot.base.bodies);
    frame.R = R0*robot.base.R;
    frame.t = t0 + R0*robot.base.t;
    frame = updateRigidBody(R0*Ri,t0 + R0*ti,frame);
    handle.robots(idx).base.R = Ri;
    handle.robots(idx).base.t = ti;
    
    % Update each coordinate frame along serial chain
    R = eye(3);
    t = robot.kin.P(:,1);
    for i=1:length(robot.kin.joint_type)
        if (robot.kin.joint_type(i) == 0 || ...
            robot.kin.joint_type(i) == 2) % rotational
            R = R*rot(robot.kin.H(:,i),q(i));
        elseif (robot.kin.joint_type(i) == 1 || ...
                robot.kin.joint_type(i) == 3) % translational
            t = t + R*robot.kin.H(:,i)*q(i);
        end
        % build 'rigidbody' structure to send to updateRigidBody
        frame.bodies = handle.bodies(robot.frames(i).bodies);
        frame.R = R0*robot.base.R*robot.frames(i).R;
        frame.t = t0 + R0*(robot.base.t + robot.base.R*robot.frames(i).t);
        frame = updateRigidBody(R0*Ri*R,t0 + R0*(ti + Ri*t),frame);
        % Store updated frame coordinates
        handle.robots(idx).frames(i).R = R;
        handle.robots(idx).frames(i).t = t;
        
        % update p for next frame
        t = t + R*robot.kin.P(:,i+1);
    end
    % Update tool frame    
    frame.bodies = handle.bodies(robot.frames(end).bodies);
    frame.R = R0*robot.base.R*robot.frames(end).R;
    frame.t = t0 + R0*(robot.base.t + robot.base.R*robot.frames(end).t);
    frame = updateRigidBody(R0*Ri*R,t0 + R0*(ti + Ri*t),frame);
    handle.robots(idx).frames(end).R = R;
    handle.robots(idx).frames(end).t = t;
    
    % Update load attached to tool if exists
    if ~isempty(robot.load)
        frame.bodies = handle.bodies(robot.load.bodies);
        frame.R = R0*robot.base.R*robot.load.R;
        frame.t = t0 + R0*(robot.base.t + robot.base.R*robot.load.t);
        Rl = R*robot.load.Rb;
        tl = t + R*robot.load.tb;
        [~] = updateRigidBody(R0*Ri*Rl, t0 + R0*(ti + Ri*tl), frame);
        % Update load frame
        handle.robots(idx).load.R = Rl;
        handle.robots(idx).load.t = tl;
    end
    
    % Store new state in handle
    handle.robots(idx).kin.state = q;
        
    % Check whether the handle is a leaf, in which case we can return, or
    % whether we need to continue into higher kinematics
    if isempty(robot.right)
        % found a leaf, can return
        return;
    else
        % pass information to 'right' kinematic branches that extend into
        % higher kinematics.
        for i=1:numel(robot.right)
            handle = updateSingleRobot(theta, handle, robot.right{i});
        end
    end
    
end