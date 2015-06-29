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

    flags = {'Color', 'FaceAlpha', 'EdgeAlpha'};
    defaults = {[0;0;0], 1, 0};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    c = opt_values{1};
    fa = opt_values{2};
    ea = opt_values{3};
    
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
    shaft.labels = attachPrefix('arrow_shaft_',shaft.labels);
    head.labels = attachPrefix('arrow_head_',head.labels);
    
    handle = combineRigidBodies(shaft,head);
end
    
    
    
    
    