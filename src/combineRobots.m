function handle = combineRobots(robot1, robot2)
    %
    % handle = combineRobots(robot1, robot2)
    %
    % combines the kinematics of robot1 and robot2 by adding the kinematics
    % of serial robot2 to the end of the kinematics of serial robot1.
    %
    % The origin for robot1 is kept as the origin for the combined robot,
    % while the end effector for robot2 is kept as the final end effector.
    % Bodies in the T frame of robot 1 are combined with bodies in the O
    % frame of robot 2
    
    % Combine kinematics
    handle.kin.H = [robot1.kin.H robot2.kin.H];
    handle.kin.P = [robot1.kin.P(:,1:end-1) robot2.kin.P];
    handle.kin.type = [robot1.kin.type robot2.kin.type];
    handle.kin.n = robot1.kin.n + robot2.kin.n;
    handle.kin.origin = robot1.kin.origin;
    
    % Combine bodies
    robot1 = attachObjectToRobot(robot1.frame(end),robot1.kin.n,robot1);
    robot1 = attachObjectToRobot(robot2.frame(1),robot1.kin.n,robot1);
    handle.frame = [robot1.frame(1:end-1) robot2.frame(2:end)];
    
    handle.load = robot2.load;
    handle.end_effector = robot2.end_effector;
end