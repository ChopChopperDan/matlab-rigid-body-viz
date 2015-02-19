function handle = createPrism(R0, t0, param, varargin)
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
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Set defaults if not already established
    if ~exist('fc','var'); fc = [1;1;1]; end
    if ~exist('fa','var'); fa = 1; end
    if ~exist('lw','var'); lw = 0.5; end
    if ~exist('ec','var'); ec = [0;0;0]; end
    if ~exist('ea','var'); ea = 1; end
    
    
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
    F1 = zeros(n,4);
    for i=1:n
        F1(i,:) = [i, mod(i,n)+1, n+mod(i,n)+1, n+i];
    end
    % top and bottom faces
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
    handle.t = t0;
    handle.labels = {'sides','polygons'};
end