function handle = createRobot(R0, t0, robot, varargin)
    %
    % handle = createRobot(R0, t0, robot,...)
    %
    % purpose: creates a robot drawing object in zero configuration
    %
    % input:
    % R0 is 3 x 3 matrix for orientation of the base frame of the robot
    % t0 is 3 x 1 vector for base frame of the robot
    % robot struct with parts:
    %       H: [ h_1 h_2 ... h_n ] actuation axis for each joint
    %       P: [p_01 p_12 p_23 .. p_{n-1}n ] inter-link vectors
    %       joint_type:     0 = rotational  1 = prismatic 
    %                       2 = rotational (non-visible) 
    %                       3 = translational (non-visible)
    %       joints: n dimensional struct with parameters 
    %                describing each joint, including a 'props' field
    %                       rotational: cylinder
    %                       prismatic:  cuboid
    %                       'non-visible' joints can be empty, 
    %                                  will not be drawn
    %       link_type: n + 1 dimensional vector describing 
    %                   each type of link
    %                       0 = no link
    %                       1 = cylindrical link
    %                       2 = cuboid link
    %       links: (n + 1) dimensional struct with parameterization 
    %               specific to each link type, including a 'props' field
    %       name: string denoting the name of the robot
    %       [opt] frame: parameters describing coordinate frames
    %
    % Optional Additional Properties:
    %       'CreateFrames'          default: 'off'
    % 
    % depends on the following drawing package files:
    %       createCuboid.m
    %       createCylinder.m
    %       create3DFrame.m
    %       attachPrefix.m
    %
    % returns handle to robot drawing object
    %
    % see also UPDATEROBOT CREATEPARALLELJAWGRIPPER CREATE3DFRAME

    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'), cf = 'off'; end
    
    % Normal orientation for actuation axis on primitive shapes
    z0 = [0;0;1];
    
    % Initialize components so that structure can be treated as a 'rigid
    % body' in other functions.
    handle.bodies = [];
    handle.labels = {};
    handle.R = eye(3);
    handle.A = eye(3);
    handle.t = [0;0;0];
    
    % set name of robot
    handle.robots.name = robot.name;
    
    
    % Transform kinematics into desired coordinate system (these terms
    %       should be constant)
    handle.robots.kin.H = R0*robot.H;
    handle.robots.kin.P = R0*robot.P;
    handle.robots.kin.P(:,1) = handle.robots.kin.P(:,1) + t0;
    handle.robots.kin.type = robot.joint_type;
    
    % Placeholders for frame and load information
    handle.robots.frames = struct('bodies',[],'R',eye(3),'t',[0;0;0]);
    handle.robots.base = struct('bodies',[],'R',eye(3),'t',[0;0;0]);
    handle.robots.load = [];
    
    % Counter for the number of bodies
    n_bodies = 0;
    
    % Create link p_01
    if robot.link_type(1) > 0
        Ri = R0*robot.links(1).R0;
        ti = t0 + R0*robot.links(1).t0;
        if robot.link_type(1) == 1
            link = createCylinder(Ri, ti, robot.links(1), ...
                                    robot.links(1).props{:});
        elseif robot.link_type(1) == 2
            link = createCuboid(Ri, ti, robot.links(1), ...
                                    robot.links(1).props{:});
        end
    else
        link.bodies = [];
        link.labels = {};
    end
    
    % Add bodies/labels to master list
    handle.bodies = [handle.bodies link.bodies];
    handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_base_'], link.labels)];
    % Add references within the 'base' frame
    handle.robots.base.bodies = 1:numel(link.bodies);
    % Increment body count
    n_bodies = n_bodies + numel(link.bodies);
        
    % Pre-compute positions of all joint frames in zero pose
    P = cumsum(handle.robots.kin.P,2);
    p = P(:,1);
    R = R0; 
    
    % Fill the terms relating bodies to each frame within the robot
    for i=1:length(handle.robots.kin.type)
        
        % Update the frame position and orientation
        handle.robots.frames(i).R = eye(3);
        handle.robots.frames(i).t = p;
        
        % Create joint i
        h = handle.robots.kin.H(:,i);
        if (abs(z0'*h)==1), Ri = eye(3);
        else                Ri = rot(hat(z0)*h, acos(h(3)));
        end
        
        if handle.robots.kin.type(i) == 0 
            % rotational
            joint = createCylinder(Ri, p, robot.joints(i), ...
                                    robot.joints(i).props{:});
            % Add bodies/labels to master list
            handle.bodies = [handle.bodies joint.bodies];
            handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_joint' num2str(i) '_'], ...
                                                        joint.labels)];
            % Add pointers to body indices in the frame subfield
            handle.robots.frames(i).bodies = n_bodies + ...
                                        (1:numel(joint.bodies));
            n_bodies = n_bodies + numel(joint.bodies);
        elseif handle.robots.kin.type(i) == 1 
            % prismatic
            joint = createPrismaticJoint(Ri, p, robot.joints(i), ...
                                    robot.joints(i).props{:});
            % Add bodies/labels to master list
            handle.bodies = [handle.bodies joint.bodies];
            handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_joint' num2str(i) '_'], ...
                                                        joint.labels)];
            
            % Add pointers to body indices in the frame subfield
            if (i > 1)
                handle.robots.frames(i-1).bodies = ...
                                [handle.robots.frames(i-1).bodies ...
                                                n_bodies + 1];
            end
            handle.robots.frames(i).bodies = n_bodies + 2;
            n_bodies = n_bodies + numel(joint.bodies);
        elseif any(handle.robots.kin.type(i) == [2 3])
            % non-visible joints aren't drawn
            handle.robots.frames(i).bodies = [];
        end
        
        % If drawing coordinate frames, draw one at joint i
        if strcmpi(cf,'on') && ~any(handle.robots.kin.type(i) == [2 3])
            frame = create3DFrame(R,p, robot.frame);
            % Add bodies/labels to master list
            handle.bodies = [handle.bodies frame.bodies];
            handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_frame' num2str(i) '_'], ...
                                                        frame.labels)];
            % Add pointers to body indices in the frame subfield
            handle.robots.frames(i).bodies = ...
                    [handle.robots.frames(i).bodies ...
                    n_bodies + (1:numel(frame.bodies))];
            n_bodies = n_bodies + numel(frame.bodies);
        end
        
        % Create link p_{i,i+1}, if specified
        if robot.link_type(i+1) > 0
            Ri = R*robot.links(i+1).R0;
            ti = p + R*robot.links(i+1).t0;
            if robot.link_type(i+1) == 1
                link = createCylinder(Ri, ti, robot.links(i+1), ...
                                robot.links(i+1).props{:});
            elseif robot.link_type(i+1) == 2
                link = createCuboid(Ri, ti, robot.links(i+1), ...
                                robot.links(i+1).props{:});
            end
        else
            link.bodies = [];
            link.labels = {};
        end
        % Add bodies/labels to master list
        handle.bodies = [handle.bodies link.bodies];
        handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_link' num2str(i) '_'], ...
                                                            link.labels)];
        % Add pointers to body indices in the frame subfield
        handle.robots.frames(i).bodies = ...
                [handle.robots.frames(i).bodies ...
                n_bodies + (1:numel(link.bodies))];
        n_bodies = n_bodies + numel(link.bodies);
        
        % Pre-computed forward kinematics
        p = P(:,i+1);
    end
    
    nj = length(handle.robots.kin.type);
    
    % Add final frame at O_T
    handle.robots.frames(nj+1).bodies = [];
    handle.robots.frames(nj+1).R = eye(3);
    handle.robots.frames(nj+1).t = p;
    
    % If drawing coordinate frames, draw one at position O_T
    if strcmpi(cf,'on')
        frame = create3DFrame(R, p, robot.frame);
        % Add bodies/labels to master list
        handle.bodies = [handle.bodies frame.bodies];
        handle.labels = [handle.labels ...
                attachPrefix([handle.robots.name '_frameT_'], frame.labels)];
        % Add pointers to body indices in the frame subfield
        handle.robots.frames(nj+1).bodies = ...
                    [handle.robots.frames(nj+1).bodies ...
                    n_bodies + (1:numel(frame.bodies))];
    end
        
end