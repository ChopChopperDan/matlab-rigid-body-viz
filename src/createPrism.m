function handle = createPrism(R0, t0, param, varargin)
    %
    % handle = createPrism(R0, t0, param,...)
    %
    % R0 is orientation of the prism relative to body frame
    % t0 is center of the prism relative to body frame
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
    if size(polygon,2) == 2 && size(polygon,1) ~= 2
        polygon = polygon';
    elseif size(polygon,1) ~= 2
        disp('end face polygon must be 2 x N or N x 2');
        handle = [];
        return;
    end
    if any(polygon(:,1) ~= polygon(:,end))
        polygon = [polygon polygon(:,1)];
    end
    
    n = size(polygon,2)-1;
    X = polygon(1,:);
    Y = polygon(2,:);
    side_faces = zeros(3,4*n);
    
    for i=1:n
        side_faces(:,(i-1)*4+1:i*4) = [X(i:i+1) X(i+1:-1:i); ...
                                       Y(i:i+1) Y(i+1:-1:i); ...
                                       h/2   h/2  -h/2  -h/2];
    end
    side_faces_rot = t0*ones(1,4*n) + R0*side_faces;
    sides_X = reshape(side_faces_rot(1,:),[4 n]);
    sides_Y = reshape(side_faces_rot(2,:),[4 n]);
    sides_Z = reshape(side_faces_rot(3,:),[4 n]);
    
    cap_faces = [polygon polygon; h/2*ones(1,n+1) -h/2*ones(1,n+1)];
    cap_faces_rot = t0*ones(1,2*(n+1)) + R0*cap_faces;
    caps_X = reshape(cap_faces_rot(1,:),[n+1 2]);
    caps_Y = reshape(cap_faces_rot(2,:),[n+1 2]);
    caps_Z = reshape(cap_faces_rot(3,:),[n+1 2]);
    
    handle.bodies(1) = patch(sides_X,sides_Y,sides_Z,1, ...
                                    'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.bodies(2) = patch(caps_X,caps_Y,caps_Z,1, ...
                                    'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.R = eye(3);
    handle.t = t0;
    handle.labels = {'sides','caps'};
end