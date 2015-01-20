function handle = createParallelJawGripper(R0, t0, param, varargin)
    % 
    % handle = createParallelJawGripper(R0, t0, param, ...)
    %
    % R0 is orientation of the gripper 
    % t0 is the bottom of the base of the gripper 
    % param is struct containing fields
    %       aperture (maximum opening between two fingers)
    %       stroke   (distance the fingers can close by (aperture - minimum
    %                   opening between fingers)
    %       height   (height of each finger)
    %       [opt] fingerwidth (width of each finger)
    % 
    % Additional parameters include:
    %       'Color':   default: [0;0;0]
    %
    % depends on the following drawing package files:
    %       createCuboid.m
    %       attachPrefix.m
    %
    % returns handle to drawing structure
    %
    % see also UPDATEPARALLELJAWGRIPPER
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0;0;0]; end
    if ~isfield(param,'fingerwidth')
        param.fingerwidth = 0.1*param.height;
    end
        
    finger_param.width = param.fingerwidth;
    finger_param.length = param.fingerwidth;
    finger_param.height = param.height;
    palm_param.width = param.aperture + 2*param.fingerwidth;
    palm_param.length = param.fingerwidth;
    palm_param.height = param.fingerwidth;
    
    props = {'FaceColor',c,'EdgeColor',c};
    
    t_lf = t0 + R0*[-(palm_param.width/2 - finger_param.width/2); 0;...
                    finger_param.height/2 + palm_param.height];
    t_rf = t0 + R0*[(palm_param.width/2 - finger_param.width/2); 0;...
                    finger_param.height/2 + palm_param.height];
    t_palm = t0 + R0*[0; 0 ;palm_param.height/2];
    
    left_jaw = createCuboid(R0,t_lf,finger_param, props{:});
    right_jaw = createCuboid(R0,t_rf,finger_param, props{:});
    middle_bar = createCuboid(R0,t_palm,palm_param, props{:});
    
    handle.bodies = [left_jaw.bodies right_jaw.bodies middle_bar.bodies];
    handle.labels = [attachPrefix('ljaw_',left_jaw.labels) ...
                        attachPrefix('rjaw_',right_jaw.labels) ...
                        attachPrefix('palm_', middle_bar.labels)];
    handle.R = eye(3);
    handle.t = t0;
    handle.gripper_info.param = param;
    handle.gripper_info.RT = R0;
    handle.gripper_info.aperture = param.aperture;
end