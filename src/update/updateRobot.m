function handle = updateRobot(theta, handle)
    %
    % handle = updateRobot(theta, handle)
    %
    % theta is a struct array containing the actuator displacements for all
    %       robots that need updating.  **Does not need to be every robot
    %       defined with the robot structure**
    %
    %       Must have the form
    %   root
    %       -> name    : string id for the robot to update 
    %       -> state   : vector containing the updated state for the robot
    %                       
    % handle is the robot drawing structure that needs updating.  Returns
    %       the updated structure.
    %
    % depends on
    %           updateRigidBody.m
    %
    % see also UPDATERIGIDBODY CREATEROBOT
    
    % check correctness of theta input
    if ~isfield(theta,'name') || ~isfield(theta,'state')
        error('updateRobot:incorrect_struct', ...
                    'theta must have fields "name" and "state"');  
    end
    
    for i=1:numel(handle.robots)
        if strcmpi(handle.robots(i).left,'root')
            handle = updateSingleRobot(theta, handle, handle.robots(i).name);
        end
    end
    
end

function handle = updateSingleRobot(theta, handle, name)
    % Calculate forward kinematics for this serial chain
    
    % Locate robot index via passed name parameter
    idx = strcmpi({handle.robots.name},name);
    robot = handle.robots(idx);
    
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
        Ri = handle.robots(left_idx).base.R * ...
                handle.robots(left_idx).frames(end).R;
        ti = handle.robots(left_idx).base.t + ...
                handle.robots(left_idx).base.R * ...
                handle.robots(left_idx).frames(end).t;
    end
    
    Rb = R0*robot.base.R;
    tb = t0 + R0*robot.base.t;
    
    Rb_i = R0*Ri;
    tb_i = t0 + R0*ti;
    
    % create dummy rigidbody struct for updateRigidBody
    body = struct('bodies',[],'R',eye(3),'t',[0;0;0]);
        
    % Extract relevant information from theta and update base frame
    theta_idx = strcmpi({theta.name},name);
    if ~any(theta_idx ~= 0) || ...
            ~any(theta(theta_idx).state ~= robot.kin.state(:))
        % If name isn't found in theta as a robot to update, or
        %   passed states match the current state, then
        %   treat the entire robot as a rigid body with respect to
        %   the base frame
        
        if ~isempty(robot.load)
            load_bodies = robot.load.bodies;
        else
            load_bodies = [];
        end
        
        body.bodies = handle.bodies([robot.base.bodies ...
                                        robot.frames.bodies ...
                                        load_bodies]);
        body.R = Rb;
        body.t = tb;
        [~] = updateRigidBody(Rb_i,tb_i,body);
        handle.robots(idx).base.R = Ri;
        handle.robots(idx).base.t = ti;
    else
        % If name is found, then need to compute forward kinematics
        q = theta(theta_idx).state;
        
        body.bodies = handle.bodies(robot.base.bodies);
        body.R = Rb;
        body.t = tb;
        [~] = updateRigidBody(Rb_i,tb_i,body);
        handle.robots(idx).base.R = Ri;
        handle.robots(idx).base.t = ti;
            
        % Update each coordinate frame along serial chain
        R = eye(3);
        t = robot.kin.P(:,1);
        for i=1:length(robot.kin.joint_type)
            % build 'rigidbody' structure to send to updateRigidBody
            body.bodies = handle.bodies(robot.frames(i).bodies);
            body.R = Rb*robot.frames(i).R;
            body.t = tb + Rb*robot.frames(i).t;
            
            if (robot.kin.joint_type(i) == 0 || ...
                robot.kin.joint_type(i) == 2) % rotational
                R = R*rot(robot.kin.H(:,i),q(i));
            elseif (robot.kin.joint_type(i) == 1 || ...
                    robot.kin.joint_type(i) == 3) % translational
                t = t + R*robot.kin.H(:,i)*q(i);
            end
            [~] = updateRigidBody(Rb_i*R,tb_i + Rb_i*t,body);
            % Store updated frame coordinates
            handle.robots(idx).frames(i).R = R;
            handle.robots(idx).frames(i).t = t;

            % update t for next frame
            t = t + R*robot.kin.P(:,i+1);
        end
        % Update tool frame    
        body.bodies = handle.bodies(robot.frames(end).bodies);
        body.R = Rb*robot.frames(end).R;
        body.t = tb + Rb*robot.frames(end).t;
        [~] = updateRigidBody(Rb_i*R,tb_i + Rb_i*t,body);
        handle.robots(idx).frames(end).R = R;
        handle.robots(idx).frames(end).t = t;

        % Update load attached to tool if exists
        if ~isempty(robot.load)
            body.bodies = handle.bodies(robot.load.bodies);
            body.R = Rb*robot.load.R;
            body.t = tb + Rb*robot.load.t;
            Rl = R*robot.load.Rb;
            tl = t + R*robot.load.tb;
            [~] = updateRigidBody(Rb_i*Rl, tb_i + Rb_i*tl, body);
            % Update load frame
            handle.robots(idx).load.R = Rl;
            handle.robots(idx).load.t = tl;
        end

        % Store new state in handle
        handle.robots(idx).kin.state = q;
    
    end
        
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