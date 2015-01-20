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
    % Depends on the following drawing package files:
    %       createCuboid.m
    %       createCylinder.m
    %       attachPrefix.m
    %
    % See Also CREATECUBOID, CREATECYLINDER
    %
    % returns handle to drawing structure
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0;0;0]; end
    
    z0 = [0;0;1];
    
    t0_lens = t0 - 1/2*param.lens.height*R0*z0;
    t0_body = t0 - (param.lens.height + 1/2*param.body.height) * R0*z0;
    
    cam_props = {'FaceColor', c, 'EdgeColor', [0.5;0.5;0.5]};
    
    body = createCuboid(R0, t0_body, param.body, cam_props{:});
    lens = createCylinder(R0, t0_lens, param.lens, cam_props{:});
    
    body.labels = attachPrefix('camera_body_', body.labels);
    lens.labels = attachPrefix('camera_lens_', lens.labels);
    handle = combineRigidBodies(body, lens);
    handle.R = eye(3);
    handle.t = t0;
end