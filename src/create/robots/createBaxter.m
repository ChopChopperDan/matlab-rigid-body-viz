    function handle = createBaxter(R0, t0, varargin)
    % 
    % h_baxter = createBaxter(R0, t0, ...)
    %
    % File to create a Baxter drawing object.  The Baxter robot 
    % consists of three serial chain robots representing the two arms and 
    % the head.  The base frame for the entire system is located at the
    % center of the waist between the body and the pedestal.
    %
    % Optional parameters for initialization are:
    %       'CreateFrames'      default: 'off'
    %       'CreatePedestal'    default: 'on'
    %
    % returns handle to the baxter structure containing three branched 
    % robots:
    %       baxter_left_arm
    %       baxter_right_arm 
    %       baxter_head
    %
    % see also DEFINEBAXTER CREATEROBOT
    
    flags = {'CreateFrames', 'CreatePedestal'};
    defaults = {'off', 'on'};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    cf = opt_values{1};
    cp = opt_values{2};
        
    zed = [0;0;0];
    
    baxter_def = defineBaxter('Origin', [R0 t0; zed' 1], 'Pedestal', cp);
    
    %%%%%% Create robot objects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h_left_arm = createRobot(eye(3), zed, baxter_def.left_arm_const,...
                                        'CreateFrames',cf);
    h_right_arm = createRobot(eye(3), zed, baxter_def.right_arm_const,...
                                        'CreateFrames',cf);
    h_head = createRobot(eye(3), zed, baxter_def.head_const,...
                                        'CreateFrames',cf);
    
    
    %%%%%% Combine into single multibranch robot handle %%%%%%%%%%%%%%%%%%%
    handle = branchRobots(h_left_arm, h_right_arm);
    handle = branchRobots(handle, h_head);
end
    