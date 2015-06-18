function handle = createMotomanSDA10D(R0, t0, varargin)
    %
    % handle = createMotomanSDA10D(R0, t0, ...)
    %
    % Function to create a Yaskawa Motoman SDA10 robot drawing object.
    %
    % Optional parameters for initialization are:
    %       'CreateFrames'      default: 'off'
    %
    % depends on the following drawing package files:
    %       createRobot.m
    %       branchRobots.m
    %       combineRobots.m
    %
    % returns handle to the Motoman SDA10 branched robot structure
    %       sda10d_torso
    %       sda10d_left_arm
    %       sda10d_right_arm
    %
    % see also DEFINEMOTOMANSDA10D
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end

    zed = [0;0;0];
    
    % Define Kinematics for the arms 
    sda10_const = defineMotomanSDA10D([R0 t0; zed' 1]);
    
    % Build individual robot structures
    h_torso = createRobot(eye(3), zed, sda10_const.torso_const, ...
                                                'CreateFrames',cf);
    h_left_arm = createRobot(eye(3), zed, sda10_const.left_arm_const, ...
                                                'CreateFrames',cf);
    h_right_arm = createRobot(eye(3), zed, sda10_const.right_arm_const, ...
                                                'CreateFrames',cf);
    
    % Build full robot structure 
    arm_offset = sum(sda10_const.torso_const.kin.P,2);
    h_left_arm = updateRigidBody(eye(3),arm_offset,h_left_arm);
    h_right_arm = updateRigidBody(eye(3),arm_offset,h_right_arm);
    h_arms = branchRobots(h_left_arm,h_right_arm);
    handle = combineRobots(h_torso,h_arms);
end