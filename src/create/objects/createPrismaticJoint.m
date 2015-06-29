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
    
    flags = {'FaceColor','FaceAlpha','LineWidth','EdgeColor','EdgeAlpha'};
    defaults = {[1;1;1], 1, 0.5, [0;0;0], 1};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    fc = opt_values{1};
    fa = opt_values{2};
    lw = opt_values{3};
    ec = opt_values{4};
    ea = opt_values{5};
        
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
    
    body.labels = attachPrefix('body_', body.labels);
    slider.labels = attachPrefix('slider_', slider.labels);
    
    handle = combineRigidBodies(body, slider);
end
    
    