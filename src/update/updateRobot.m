function handle = updateRobot(theta, handle)
    %
    % handle = updateRobot(theta, handle)
    %
    % theta is a cell containing vectors for the joint displacements 
    %       of all robots within the handle
    %    theta = { [robot 1 joint angles] 
    %                     ... 
    %              [robot n joint angles] }
    %       -> in the case of only a single chain robot, can just be a
    %       single vector
    %    theta = [robot joint angles]
    %       -> in the case of branched robots, should have nested cells
    %               expected in correct kinematic order.
    %    theta = { [robot 1 joint angles]
    %                 { [robot 2 joint angles]
    %                   [robot 2_1 joint angles]
    %                   { [robot 2_2 joint angles] 
    %                        [robot 2_2_1 joint angles]
    %                        [robot 2_2_2 joint angles]
    %                        [robot 2_2_3 joint angles] } }
    %                ...
    %              [robot n joint_angles] }
    %       -> in the case of branched robots, should have nested cells
    %               with the frame name as the identifier.
    %    theta = { robot 1 id, [robot 1 joint angles]
    %                { robot 2 id, [robot 2 joint angles]
    %                  robot 2_1 id, [robot 2_1 joint angles]
    %                    { robot 2_2 id, [robot 2_2 joint angles] 
    %                      robot 2_2_1 id, [robot 2_2_1 joint angles]
    %                      robot 2_2_2 id, [robot 2_2_2 joint angles]
    %                      robot 2_2_3 id, [robot 2_2_3 joint angles] } }
    %                 ...
    %              robot n id, [robot n joint_angles] }
    %                       
    % handle is handle to drawing structure containing a 'robots' field.  
    %
    % depends on
    %           updateRigidBody.m
    %
    % returns updated handle
    %
    % see also UPDATERIGIDBODY CREATEROBOT
    
    % leaf node within robot structure, exit case
    if ~isfield(handle,'robots') || isempty(handle.robots)
        return; 
    end
    
    if iscell(theta)
        % Step through root for robots and send their information into the 
        %   recursive function
        for i=1:numel(handle.robots)
            handle.robots(i) = updateRobotRecursive(handle.robots(i), ...
                                                handle.bodies, ...
                                                theta{i}, ...
                                                handle.R, handle.t);
        end
    else
        % Assume single serial-arm robot
        handle.robots = updateRobotRecursive(handle.robots, ...
                                                handle.bodies, ...
                                                theta, ...
                                                handle.R, handle.t);
    end
    
end

function handle = updateRobotRecursive(handle, bodies, theta, R0, t0)
    % Calculate forward kinematics for this serial chain
    %   If robot branches further, expecting a cell object for theta
    %   If robot is a leaf, expecting a simple array of displacements
    if iscell(theta)
        q = theta{1};
    else
        q = theta;
    end
    % Compute forward kinematics, update the bodies associated with each
    % frame, and store the local transformations to the parent
    % compute forward kinematics and update bodies attached to each 
    %   joint's frame
    
    % update bodies attached to the base frame
    frame.bodies = bodies(handle.base.bodies);
    frame.R = handle.base.R;
    frame.t = handle.base.t;
    frame = updateRigidBody(R0,t0,frame);
        
    R = eye(3);
    t = handle.kin.P(:,1);
    for i=1:length(handle.kin.type)
        if any(handle.kin.type(i) == [0 2]) % rotational
            R = R*rot(handle.kin.H(:,i),q(i));
        elseif any(handle.kin.type(i) == [1 3]) % prismatic
            t = t + R*handle.kin.H(:,i)*q(i);
        end
        % build 'rigidbody' structure to send to updateRigidBody
        frame.bodies = bodies(handle.frames(i).bodies);
        frame.R = handle.base.R*handle.frames(i).R;
        frame.t = handle.base.t + handle.base.R*handle.frames(i).t;
        frame = updateRigidBody(R0*R,t0 + R0*t,frame);
        % Store local transformation from base frame of robot
        handle.frames(i).R = R;
        handle.frames(i).t = t;
        
        % update p for next frame
        t = t + R*handle.kin.P(:,i+1);
    end
    % Update tool frame
    frame.bodies = bodies(handle.frames(end).bodies);
    frame.R = handle.base.R*handle.frames(end).R;
    frame.t = handle.base.t + handle.base.R*handle.frames(end).t;
    frame = updateRigidBody(R0*R,t0 + R0*t,frame);
    
    % Update load attached to tool if exists
    if ~isempty(handle.load)
        frame.bodies = bodies(handle.load.bodies);
        frame.R = handle.base.R*handle.frames(end).R*handle.load.Rb;
        frame.t = handle.base.t + handle.base.R*(handle.frames(end).t + ...
                            handle.frames(end).R*handle.load.tb);
        RL = R0*R*handle.load.Rb;
        tL = t0 + R0*(t + R*handle.load.tb);
        frame = updateRigidBody(RL, tL, frame);
        % Update local transformation from base frame
        handle.load.R = R0*R*handle.load.Rb;
        handle.load.t = t0 + R0*(t + R*handle.load.tb);
    end
    % Update local transformation from base frame for tool frame
    handle.frames(end).R = R;
    handle.frames(end).t = t;
    
    % Update transformation from root frame to robot base frame
    handle.base.R = R0;
    handle.base.t = t0;
    
    % Check whether the handle is a leaf, in which case we can return, or
    % whether we need to step down to a lower branch
    if ~isfield(handle,'robots') || isempty(handle.robots)
        % found a leaf, can return
        return;
    else
        % pass information to lower branch and update the transformation
        % terms of the lower branch to reflect the local transformation
        % from the base frame to the end of this serial chain
        for i=1:numel(handle.robots)
            handle.robots(i) = updateRobotRecursive(handle.robots(i), ...
                                                    bodies, ...
                                                    theta{i+1}, ...
                                                    R0*R, t0+R0*t);
        end
    end
    
end