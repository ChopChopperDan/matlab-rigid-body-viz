function handle = createCamera(R0, t0, param, varargin)
    %
    % handle = createCamera(R0, t0, param, ...)
    %
    % R0 is the 3x3 orientation matrix of the camera
    % t0 is the 3x1 position of the lens of the camera
    % param is struct containing fields
    %       body (cuboid parameterization)
    %       lens (cylinder parameterization)
    %
    % Additional parameters include:
    %       'Color':    default: [0;0;0]
    %
    % See Also CREATECUBOID, CREATECYLINDER
    %
    % returns handle to drawing structure
    
    flags = {'Color'};
    defaults = {[0;0;0]};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    c = opt_values{1};
        
    z0 = [0;0;1];
    
    t0_lens = t0 - 1/2*param.lens.height*R0*z0;
    t0_body = t0 - (param.lens.height + 1/2*param.body.height) * R0*z0;
    
    cam_props = {'FaceColor', c, 'EdgeColor', [0.5;0.5;0.5]};
    
    body = createCuboid(R0, t0_body, param.body, cam_props{:});
    lens = createCylinder(R0, t0_lens, param.lens, cam_props{:});
    
    body.labels = attachPrefix('camera_body_', body.labels);
    lens.labels = attachPrefix('camera_lens_', lens.labels);
    handle = combineRigidBodies(body, lens);
end