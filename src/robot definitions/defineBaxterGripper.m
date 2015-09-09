function [gripper_const, gripper_structure] = ...
                                    defineBaxterGripper(param, varargin)
    % DEFINEBAXTERGRIPPER
    %
    % [gripper_const, gripper_structure] = defineBaxterGripper(param)
    %       param is struct containing fields
    %           -> aperture (maximum opening between two fingers
    %           -> height   (height of each finger)
    % [gripper_const, gripper_structure] = defineBaxterGripper(param, ...)
    %       allows additional optional parameters
    %
    %           'Origin'     :  default [eye(3) [0;0;0]; [0 0 0] 1]
    %           'Name'       :  default 'gripper'
    %
    % returns:
    %       gripper_const - struct array for the two 'robots' that make up 
    %           a generic parallel-jaw gripper
    %       gripper_structure - struct array for when creating combined
    %           robots
    %
    % see also CREATECOMBINEDROBOT, DEFINEPARALLELJAWGRIPPER
    
    flags = {'Origin', 'Name'};
    defaults = {eye(4), 'gripper'};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    name = opt_values{2};
    
    % set paramaters / properties for parallel jaw gripper
    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    origin = origin*[rot([0;1;0],pi/2)*rot([0;0;1],pi/2) ...
                                [0.04;0;0]; [0 0 0 1]];
    param.fingerwidth = 0.01;
    
    [gripper_const, ...
        gripper_structure] = defineParallelJawGripper(param, ...
                                                    'Origin', origin, ...
                                                    'Name', name);
    
    % attach cylinder base to palm
    gripper_const(1).vis.peripherals.id = 'palm_enclosure';
    gripper_const(1).vis.peripherals.frame = 'base';
    gripper_const(1).vis.peripherals.handle = @createCylinder;
    gripper_const(1).vis.peripherals.R = R0*rot([1;0;0],pi/2);
    gripper_const(1).vis.peripherals.t = t0 + R0*[0.025;0;0];
    gripper_const(1).vis.peripherals.param = struct('radius',0.025, ...
                                                    'height', 0.1);
    gripper_const(1).vis.peripherals.props = {'FaceColor',[0.9;0;0]};
end