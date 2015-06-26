function handle = createPrism(R0, t0, param, varargin)
    % CREATEPRISM
    %
    % handle = createPrism(R0, t0, param,...)
    %
    % R0 is 3 x 3 matrix for orientation of the prism
    % t0 is 3 x 1 vector for center of the prism
    % param is struct containing fields
    %       polygon (ordered series of 2D points for end faces)
    %       height [Z axis]
    % possible additional properties are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % returns handle to drawing structure
    %
    % see also CREATECYLINDER CREATECUBOID CREATEELLIPSOID
    
    flags = {'FaceColor','FaceAlpha','LineWidth','EdgeColor','EdgeAlpha'};
    defaults = {[1;1;1], 1, 0.5, [0;0;0], 1};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    fc = opt_values{1};
    fa = opt_values{2};
    lw = opt_values{3};
    ec = opt_values{4};
    ea = opt_values{5};
    
    polygon = param.polygon;
    h = param.height;
    if size(polygon,1) == 2 && size(polygon,2) ~= 2
        polygon = polygon';
    elseif size(polygon,1) ~= 2
        disp('end face polygon must be 2 x N or N x 2');
        handle = [];
        return;
    end
    
    % Vertices
    n = size(polygon,1);
    V = [polygon h/2*ones(n,1); polygon -h/2*ones(n,1)];
    V = V*R0' + ones(2*n,1)*t0';
    
    % Faces
    % side faces
    F = NaN*ones(n+2,n);
    for i=1:n
        F(i,1:4) = [i, mod(i,n)+1, n+mod(i,n)+1, n+i];
    end
    % top and bottom faces
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
    handle.labels = {'sides','polygons'};
end