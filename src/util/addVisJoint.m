function robot_const = addVisJoint(robot_const, frame, param, props)
    % ADDVISJOINT
    %
    % robot_const = addVisJoint(robot_const, frame, param, props)
    
    
    robot_const.vis.joints(frame) = struct('param', param, 'props', {props});
end