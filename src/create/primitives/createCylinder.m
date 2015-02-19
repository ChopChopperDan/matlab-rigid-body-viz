function handle = createCylinder(R0, t0, param, varargin)
    %
    % handle = createCylinder(R0, t0, param, ...)
    %
    % R0 is 3 x 3 matrix for orientation of the cylinder 
    % t0 is 3 x 1 vector for center of the cylinder
    % param is struct containing fields
    %       radius 
    %       height 
    %       [opt] radius2 
    %       [opt] radiusX
    %       [opt] radiusY
    %       [opt] radiusX2
    %       [opt] radiusY2
    %
    % possible additional properties are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % returns handle to drawing structure
    %
    % see also CREATECUBOID CREATEELLLIPSOID CREATEPRISM
    
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
            error(['Property not recognized: ' varargin{i}]);
        end
    end
    % Set defaults if not already established
    if ~exist('fc','var'); fc = [1;1;1]; end
    if ~exist('fa','var'); fa = 1; end
    if ~exist('lw','var'); lw = 0.5; end
    if ~exist('ec','var'); ec = [0;0;0]; end
    if ~exist('ea','var'); ea = 1; end
    
    % Verify parameters are correct
    invalid = false;
    if isfield(param,'height')
        h = param.height;
    else
        invalid = true;
    end
    if isfield(param,'radius')
        rx = param.radius;
        ry = param.radius;
    elseif isfield(param,'radiusX') && isfield(param, 'radiusY')
        rx = param.radiusX;
        ry = param.radiusY;
    else
        invalid = true;
    end
    
    if isfield(param,'radius2')
        rx2 = param.radius2;
        ry2 = param.radius2;
    elseif isfield(param,'radiusX2') && isfield(param, 'radiusY2')
        rx2 = param.radiusX2;
        ry2 = param.radiusY2;
    else
        rx2 = rx;
        ry2 = ry;
    end
    
    if invalid
        disp('Valid parameterizations are:');
        disp('    height, radius - for circular cylinder');
        disp('    height, radius, radius2 -  for cone');
        disp('    height, radiusX, radiusY - for elliptic cylinder');
        disp('    height, radiusX, radiusY, radiusX2, radiusY2 - for elliptic cone');
        error('Invalid Parameterization for cylinder');
    end
    
    % Vertices
    n = 20; % resolution for circle discretization
    X = cos(2*pi*(0:1/n:(n-1)/n));
    Y = sin(2*pi*(0:1/n:(n-1)/n));
    Z = ones(1,n);
    
    % Must be nv x 3 for patch
    V = [rx*X rx2*X; ry*Y ry2*Y; h/2*Z -h/2*Z]';
    V = V*R0' + ones(2*n,1)*t0';
    
    % Faces
    
    % side faces
    F1 = zeros(n,4);
    for i=1:n
        F1(i,:) = [i mod(i,n)+1 n+mod(i,n)+1 n+i];
    end
    
    % end caps
    F2 = [1:n;n+1:2*n];
    
    FV_sides.Vertices = V;
    FV_sides.Faces = F1;
    FV_caps.Vertices = V;
    FV_caps.Faces = F2;
    handle.bodies(1) = patch(FV_sides, 'FaceColor',fc, ...
                                        'FaceAlpha',fa, ...
                                        'LineWidth',lw, ...
                                        'EdgeColor',ec, ...
                                        'EdgeAlpha',ea);
    handle.bodies(2) = patch(FV_caps, 'FaceColor',fc, ...
                                        'FaceAlpha',fa, ...
                                        'LineWidth',lw, ...
                                        'EdgeColor',ec, ...
                                        'EdgeAlpha',ea);
    handle.R = eye(3);
    handle.t = [0;0;0];
    handle.A = eye(3);
    handle.labels = {'sides','caps'};
end