function kin = getKinematicChains(handle)
    % 
    % kin = getKinematicChains(handle)
    %
    % finds the combined kinematic chains to the tool frame on each robot
    % and all nested robots
    %
    % returns structure of form
    %       kin.H 
    %       kin.P
    %       kin.type
    %       kin.robots.H <-contains kinematics of nested robot and top
    %                           level robot
    %       kin.robots.P
    %       kin.robots.type
    %       kin.robots.robots.H
    %           ...
    %
    
    % initialize 
    kin = struct('H',[],'P',[],'type',[],'robots',[],'name',[]);
    for i=1:numel(handle.robots)
        kin(i) = recursiveKinematicSearch(handle.robots(i),[]);
        
    end
end

function kin = recursiveKinematicSearch(robot,prior_kin)
    
    if isempty(prior_kin)
        kin = robot.kin;
        kin.robots = struct('H',[],'P',[],'type',[],'robots',[],'name',[]);
    else
        kin = prior_kin;
        kin.H = [kin.H robot.kin.H];
        kin.type = [kin.type;robot.kin.type(:)];
        kin.P = [kin.P(:,1:end-1) kin.P(:,end)+robot.kin.P(:,1) ...
                                                robot.kin.P(:,2:end)];
    end
    kin.name = robot.name;
    disp(robot.name);
    disp(kin);

    % Check whether the handle is a leaf, in which case we can return, or
    % whether we need to step down to a lower branch
    if ~isfield(robot,'robots') || isempty(robot.robots)
        % found a leaf, can return
        kin.robots = [];
        disp('struct returned');
        disp(kin)
        return;
    else
        % pass information to lower branch
        disp('struct before');
        disp(kin.robots)
        for i=1:numel(robot.robots)
            kin.robots(i) = recursiveKinematicSearch(robot.robots(i),kin);
        end
    end
end