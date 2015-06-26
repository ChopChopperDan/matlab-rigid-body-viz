function handle = createCylinder(R0, t0, param, varargin)
    % CREATECYLINDER
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
    
    flags = {'FaceColor','FaceAlpha','LineWidth','EdgeColor','EdgeAlpha'};
    defaults = {[1;1;1], 1, 0.5, [0;0;0], 1};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    fc = opt_values{1};
    fa = opt_values{2};
    lw = opt_values{3};
    ec = opt_values{4};
    ea = opt_values{5};
    
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
        error('createCylinder:invalid_param', ...
                'Invalid Parameterization for cylinder');
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
    F = NaN*ones(n+2,n);
    for i=1:n
        F(i,1:4) = [i mod(i,n)+1 n+mod(i,n)+1 n+i];
    end
    
    % end caps
    F(n+1:n+2,:) = [1:n;n+1:2*n];
    
    FV.Vertices = V;
    FV.Faces = F;
    handle.bodies(1) = patch(FV, 'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.R = eye(3);
    handle.t = [0;0;0];
    handle.A = eye(3);
    handle.labels = {'sides'};
end