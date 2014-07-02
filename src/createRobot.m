function handle = createRobot(robot,varargin)
    %
    % handle = createRobot(robot,...)
    %
    % purpose: creates a robot drawing object in zero configuration
    %
    % input:
    % robot struct with parts:
    %       type:   0 = rotational  1 = prismatic 
    %               2 = rotational (for mobile) 
    %               3 = translational (for mobile)
    %       H = [ h1 h2 ... hn ] axis of rotation or translation
    %       P = [p01 p12 p23 .. p_{n-1}n ] inter-link vectors
    %       n: # of joints 
    %       origin: 4 x 4 transformation matrix denoting base frame
    %       joint: n dimensional struct with parameters 
    %                   describing each joint
    %           rotational joint: radius, height
    %           prismatic joint:  width, length, height *NOTE: HEIGHT IS
    %                   ASSUMED TO BE MAXIMUM DISPLACEMENT FOR JOINT
    %           'mobile' joints: can be empty, will not be drawn
    %       link_type: n + 1 dimensional vector describing 
    %                   each type of link
    %               0 = no link
    %               1 = cylindrical link
    %               2 = cuboid link
    %       link: n + 1 dimensional struct with parameters 
    %               describing each link
    %       [opt] frame: parameters describing coordinate frames, if drawn
    %       [opt] gripper: parameters describing the gripper, if drawn
    %               *based on ParallelJawGripper parameterization
    %
    % Optional Additional Properties:
    %       'CreateFrames'          default: 'off'
    %       'CreateGripper'         default: 'on'
    % 
    % depends on the following drawing package files:
    %       createCuboid.m
    %       createCylinder.m
    %       createParallelJawGripper.m
    %       create3DFrame.m
    %       attachPrefix.m
    %
    % returns handle to robot drawing object
    %
    % see also UPDATEROBOT CREATEPARALLELJAWGRIPPER CREATE3DFRAME

    % Walk through varargin
    for i=1:2:(nargin-1)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        elseif strcmp(varargin{i},'CreateGripper')
            cg = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end
    if ~exist('cg','var'); cg = 'on'; end
    
    % Default actuation axis for joints
    z0 = [0;0;1];
    
    % Parse out the robot object
    handle.kin.type = robot.type;
    handle.kin.H = robot.origin(1:3,1:3)*robot.H;
    handle.kin.P = robot.origin(1:3,1:3)*robot.P;
    handle.kin.P(:,1) = handle.kin.P(:,1) + robot.origin(1:3,4);
    handle.kin.n = robot.n;
    handle.kin.origin = robot.origin;
    
    p = handle.kin.origin(1:3,4);
    R = handle.kin.origin(1:3,1:3);
    
    % Create link p01
    if robot.link_type(1) > 0
        R0 = robot.link(1).R0;
        t0 = p + robot.link(1).t0;
        if robot.link_type(1) == 1
            link = createCylinder(R0, t0, ...
                    robot.link(1), robot.link(1).props{:});
        elseif robot.link_type(1) == 2
            link = createCuboid(R0, t0, ...
                    robot.link(1), robot.link(1).props{:});
        end
    else
        link.bodies = [];
        link.labels = {};
    end
    handle.frame(1).bodies = link.bodies;
    handle.frame(1).labels = attachPrefix('link0_',link.labels);
    handle.frame(1).R = eye(3);
    handle.frame(1).t = p;
    
    % Positions of all joints in zero pose
    P = cumsum(handle.kin.P,2);
    p = P(:,1);
    
    for i=1:handle.kin.n
        
        handle.frame(i+1).R = eye(3);
        handle.frame(i+1).t = p;
        
        % Create joint i  
        h = handle.kin.H(:,i);
        if abs(z0'*h)==1
            Rj = eye(3);
        else
            phi = asin(norm(hat(z0)*h));
            if z0'*h < 0; phi = -phi; end
            Rj = rot(hat(z0)*h,phi);
        end
        
        if handle.kin.type(i) == 0 
            % rotational
            joint = createCylinder(Rj, p, ...
                            robot.joint(i), robot.joint(i).props{:});
            handle.frame(i+1).bodies = joint.bodies;
            handle.frame(i+1).labels = ...
                attachPrefix(['joint' num2str(i) '_'],joint.labels);
        elseif handle.kin.type(i) == 1 
            % prismatic
            joint = createPrismaticJoint(Rj, p, ...
                            robot.joint(i), robot.joint(i).props{:});
            joint.labels = attachPrefix(['joint' num2str(i) '_'], ...
                                joint.labels);
            % attach body to previous frame
            handle.frame(i).bodies = [handle.frame(i).bodies ...
                                    joint.bodies(1)];
            handle.frame(i).labels = [handle.frame(i).labels ...
                                    joint.labels(1)];
            % attach slider to current frame
            handle.frame(i+1).bodies = joint.bodies(2);
            handle.frame(i+1).labels = joint.labels(2);
        elseif handle.kin.type(i) == 2 || handle.kin.type(i) == 3
            % mobile 'joints' aren't drawn
            handle.frame(i+1).bodies = [];
            handle.frame(i+1).labels = {};
        end
        
        % If drawing coordinate frames, draw one at joint i
        if strcmpi(cf,'on') && ...
                handle.kin.type(i) ~=2 && handle.kin.type(i) ~= 3
            frame = create3DFrame(R, p, robot.frame);
            handle.frame(i+1).bodies = ...
                [handle.frame(i+1).bodies frame.bodies];
            handle.frame(i+1).labels = ...
                [handle.frame(i+1).labels ...
                    attachPrefix(['frame' num2str(i) '_'],frame.labels)];
        end
        
        % Create link p_{i,i+1}, if exists
        if robot.link_type(i+1) > 0
            R0 = R*robot.link(i+1).R0;
            t0 = p + R*robot.link(i+1).t0;
            if robot.link_type(i+1) == 1
                link = createCylinder(R0, t0, ...
                                robot.link(i+1), robot.link(i+1).props{:});
            elseif robot.link_type(i+1) == 2
                link = createCuboid(R0, t0, ...
                                robot.link(i+1), robot.link(i+1).props{:});
            end
        else
            link.bodies = [];
            link.labels = {};
        end
        handle.frame(i+1).bodies = [handle.frame(i+1).bodies link.bodies];
        handle.frame(i+1).labels = [handle.frame(i+1).labels ...
                    attachPrefix(['link' num2str(i) '_'],link.labels)];
        
        % Pre-computed forward kinematics
        p = P(:,i+1);
    end
    
    % Add final frame at O_T
    handle.frame(handle.kin.n+2).bodies = [];
    handle.frame(handle.kin.n+2).labels = {};
    handle.frame(handle.kin.n+2).R = eye(3);
    handle.frame(handle.kin.n+2).t = p;
    
    % Add container for potential load
    handle.load = [];
    
    % If drawing coordinate frames, draw one at position O_T
    if strcmpi(cf,'on')
        frame = create3DFrame(R, p, robot.frame);
        handle.frame(handle.kin.n+2).bodies = ...
                [handle.frame(handle.kin.n+2).bodies frame.bodies];
        handle.frame(handle.kin.n+2).labels = ...
                [handle.frame(handle.kin.n+1).labels ...
                attachPrefix('frameT_',frame.labels)];
    end
    
    % If drawing generic parallel jaw gripper, draw at O_T
    if strcmpi(cg,'on')
        RT = R*robot.gripper.R0;
        handle.end_effector = createParallelJawGripper(RT,p,robot.gripper);
    else
        handle.end_effector = [];
    end
    
end