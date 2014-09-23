function handle = createKinectV2(R0, t0, param, varargin)
    % CREATEKINECTV2
    %   handle = createKinectV2(R0, t0, ...)
    %
    % R0 is the 3x3 orientation matrix for the Kinect
    % t0 is the 3x1 origin of the Kinect in the world frame
    %           * note that the origin of the Kinect is with respect to its
    %           "Camera Space", which is located at the lens of the
    %           infrared camera and is oriented with +Z going down the
    %           optical axis, +Y facing up through the top of the body, and
    %           +X pointing down the width-wise direction, consistent with 
    %           right-hand coordinate frame formulation.
    %
    % param is a struct containing the fields:
    %       [None - placeholder to keep consistent with rest of library]
    %
    % Additional Parameters include:
    %       'Color':        default: [0;0;0]
    %       'Alpha':        default: 1
    %
    % depends on the following drawing package files:
    %       createCuboid.m
    %       createCylinder.m
    %       attachPrefix.m
    %
    % see also CREATECUBOID, CREATECYLINDER
    %
    % returns handle to drawing structure 
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        elseif strcmp(varargin{i},'Alpha')
            a = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0;0;0]; end
    if ~exist('a','var'); a = 1; end
    
    % define body parameterization
    base_param.width = .25;
    base_param.length = .065;
    base_param.height = .044;
    lens_param.radius = .015;
    lens_param.height = .002;
    
    % These values were determined from a calibration process
    R_base = R0*rot([1;0;0],pi/2);
    t_base = t0 + R_base*[0.0444;-0.0218;-0.0014];
    t_lens = t0 + R0*[-.0475;-0.0015;0.01];
    
    base_props = {'FaceColor', c, 'EdgeColor', [0.5;0.5;0.5], ...
                    'FaceAlpha', a, 'EdgeAlpha', a};
    
    lens_props = {'FaceColor', [0.5;0.5;0.5], ...
                    'EdgeColor', [0.5;0.5;0.5], ...
                    'FaceAlpha', a, 'EdgeAlpha', a};
    
    base = createCuboid(R_base,t_base, base_param, base_props{:});
    lens = createCylinder(R0, t_lens, lens_param, lens_props{:});
    
    base.labels = attachPrefix('KinectV2_base_', base.labels);
    lens.labels = attachPrefix('KinectV2_lens_', lens.labels);
    
    handle = combineRigidBodies(base, lens);
    handle.R = eye(3);
    handle.t = t0;
end
    
    
    
    