function handle = createArrow(R0, t0, param, varargin)
% CREATEARROW    
%   handle = createArrow(R0, t0, param, ...)
%
%   R0 is the 3x3 orientation matrix of the Arrow
%   t0 is the 3x1 position of the tail of the arrow
%   param is a struct containing the fields
%       radius - radius of arrow body
%       length - length of entire arrow
%       head_radius - radius of arrow head
%       head_height - height of arrow head
%
%   Additional parameters include:
%       'Color':        default: [0;0;0]
%       'FaceAlpha':    default: 1
%       'EdgeAlpha':    default: 1
%   
%   Depends on the following drawing package files:
%       createCylinder.m
%       attachPrefix.m
%       combineRigidBodies.m
%
%   see also CREATECYLINDER
%
%   retursn handle to drawing structure

    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        elseif strcmp(varargin{i},'FaceAlpha')
            fa = varargin{i+1};
        elseif strcmp(varargin{i},'EdgeAlpha')
            ea = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0;0;0]; end
    if ~exist('fa','var'); fa = 1; end
    if ~exist('ea','var'); ea = 1; end
    
    z0 = [0;0;1];
    
    shaft_param.radius = param.radius;
    shaft_param.height = param.length - param.head_height;
    t_shaft = t0 + shaft_param.height/2*R0*z0;
    
    head_param.radius2 = param.head_radius;
    head_param.radius = 0;
    head_param.height = param.head_height;
    t_head = t0 + (shaft_param.height + head_param.height/2)*R0*z0;
    
    arrow_props = {'FaceColor',c,'EdgeColor',[0;0;0], ...
                    'FaceAlpha',fa,'EdgeAlpha',ea};
    
    shaft = createCylinder(R0,t_shaft,shaft_param,arrow_props{:});
    head = createCylinder(R0,t_head,head_param,arrow_props{:});
    shaft.labels = attachPrefix('shaft_',shaft.labels);
    head.labels = attachPrefix('head_',head.labels);
    
    handle = combineRigidBodies(shaft,head);
    
    handle.R = eye(3);
    handle.t = t0;
end
    
    
    
    
    