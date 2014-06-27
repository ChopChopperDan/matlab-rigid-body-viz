function robot = updateRobot(theta, robot)
    %
    % robot = updateRobot(theta, robot)
    %
    % theta is vector desired joint displacements for robot
    % robot is handle to drawing structure for robot.  
    %
    % depends on updateRigidBody.m
    %
    % returns updated robot drawing structure
    %
    % see also UPDATERIGIDBODY CREATEROBOT
    
    % compute forward kinematics and update bodies at each joint's frame
    R = eye(3);
    p = robot.kin.P(:,1);
    n = robot.kin.n;
    for i=1:n
        if robot.kin.type(i) == 0 || robot.kin.type(i) == 2 % rotational
            R = R*rot(robot.kin.H(:,i),theta(i));
        elseif robot.kin.type(i) == 1 || robot.kin.type(i) == 3 % prismatic
            p = p + R*robot.kin.H(:,i)*theta(i);
        end
        robot.frame(i+1) = updateRigidBody(R,p,robot.frame(i+1));
        p = p + R*robot.kin.P(:,i+1);
    end
    % Update tool frame
    robot.frame(n+2) = updateRigidBody(R, p, robot.frame(n+2));
    % Update end effector if exists
    if ~isempty(robot.end_effector)
        robot.end_effector = updateRigidBody(R, p, robot.end_effector);
    end
    % Update load if exists
    if ~isempty(robot.load)
        RL = R*robot.load.Rb;
        pL = p + R*robot.load.tb;
        robot.load = updateRigidBody(RL, pL, robot.load);
    end
end