function handle = createPrismaticJoint(R0, t0, param, varargin)
    % 
    % handle = createPrismaticJoint(R0, t0, param, ...)
    %
    % R0 is orientation of the gripper relative to body frame
    % t0 is base of the gripper relative to body frame
    % param is struct containing fields
    %       width of body
    %       length of body
    %       height of body
    %       [opt] sliderscale (must be in [0,1]) - defines relative scale
    %           between body and slider cross sections
    % 
    % Additional properties include:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % depends on the following drawing package files:
    %       createCuboid.m
    %       attachPrefix.m
    %
    % returns handle to drawing structure
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'FaceColor')
            fc = varargin{i+1};
        elseif strcmp(varargin{i},'FaceAlpha')
            fa = varargin{i+1};
        elseif strcmp(varargin{i},'LineWidth')
            lw = varargin{i+1};
        elseif strcmp(varargin{i},'EdgeColor')
            ec = varargin{i+1};
        elseif strcmp(varargin{i},'EdgeAlpha')
            ea = varargin{i+1};
        else
            if ischar(varargin{i})
                error(['Parameter not recognized: ' varargin{i}]);
            else
                error(['Additional properties must be given in form: ' ...
                    'createPrismaticJoint(R0, t0, param,' ...
                    ' ''Property'', value)']);
            end
        end
    end
    
    % Set defaults
    if ~exist('fc','var'); fc = [1;1;1]; end
    if ~exist('fa','var'); fa = 1; end
    if ~exist('lw','var'); lw = 0.5; end
    if ~exist('ec','var'); ec = [0;0;0]; end
    if ~exist('ea','var'); ea = 1; end
    
    body_param = param;
    slider_param = param;
    if isfield(param,'sliderscale')
        slider_param.width = param.sliderscale*slider_param.width;
        slider_param.length = param.sliderscale*slider_param.length;
    end
    
    body_props = {'FaceColor', fc, ...
                    'FaceAlpha', fa, ...
                    'LineWidth', lw, ...
                    'EdgeColor', ec, ...
                    'EdgeAlpha', ea};
    slider_props = {'FaceColor', 1 - fc, ...
                    'FaceAlpha', fa, ...
                    'LineWidth', lw, ...
                    'EdgeColor', ec, ...
                    'EdgeAlpha', ea};
                    
    body = createCuboid(R0, t0, body_param, body_props{:});
    slider = createCuboid(R0, t0, slider_param, slider_props{:});
    
    handle.bodies = [body.bodies slider.bodies];
    handle.labels = [attachPrefix('body_',body.labels) ...
                        attachPrefix('slider_',slider.labels)];
    handle.R = eye(3);
    handle.t = t0;
end
    
    