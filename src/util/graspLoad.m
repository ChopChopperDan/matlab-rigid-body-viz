function [handle, robot] = graspLoad(object, handle, robot)
    %
    % handle = graspLoad(object, handle)
    % [handle, robot] = graspLoad(object, handle, robot)
    %
    % object is a handle to a drawing structure for grasp.  If load already
    %   exists, will not attempt grasp
    % handle contains full robot structure
    % [opt] robot is handle to nested robot structure to attach load to
    %       * assumes first indexed robot in handle if not supplied
    %
    % returns handle to updated robot structure with object and local
    %       transformation added to the robot.load field
    
    if nargin == 2,   robot = handle.robots(1);   end
    if ~isempty(robot.load),  return;   end

    robot.load.bodies = numel(handle.bodies) + (1:numel(object.bodies));
    robot.load.R = robot.base.R'*object.R;
    robot.load.t = robot.base.R'*(object.t - robot.base.t);
    RT = robot.base.R*robot.frames(end).R;
    tT = robot.base.t + robot.base.R*robot.frames(end).t;
    robot.load.Rb = RT'*object.R;
    robot.load.tb = RT' * (object.t - tT);
    handle.bodies = [handle.bodies object.bodies];
    handle.labels = [handle.labels object.labels];
    if nargin == 2 && nargout == 1
        handle.robots(1) = robot;
    end
end