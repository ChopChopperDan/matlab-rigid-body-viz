function handle = createCuboid(R0, t0, param, varargin)
    %
    % handle = createCuboid(R0, t0, param,...)
    %
    % R0 is 3 x 3 matrix for orientation of the cuboid
    % t0 is 3 x 1 vector for center of the cuboid
    % param is struct containing fields
    %       width [X axis]
    %       length [Y axis]
    %       height [Z axis]
    %       [opt] width2 [X axis on -Z face]
    %       [opt] length2 [Y axis on -Z face]
    % possible additional properties are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % returns handle to drawing structure
    %
    % see also CREATECYLINDER CREATEELLIPSOID CREATEPRISM
    
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
    
    % verify parameters are correct
    if isfield(param,'width') && ...
            isfield(param,'length') && ...
            isfield(param,'height')
        w = param.width; l = param.length; h = param.height;
        if isfield(param,'width2')
            w2 = param.width2;
        else
            w2 = w;
        end
        if isfield(param,'length2')
            l2 = param.length2;
        else
            l2 = l;
        end
    else
        disp('Valid parameterizations are:');
        disp('    width, length, height - for cuboid');
        disp('    [optional parameters]:');
        disp('    width2, length2 for trapezoidal faces');
        error('Invalid Parameterization for cuboid');
    end
    
    % Vertices
    V = 0.5*[-w2, -l2, -h; ...
              -w,  -l,  h; ...
             -w2,  l2, -h; ...
              -w,   l,  h; ...
              w2, -l2, -h; ...
               w,  -l,  h; ...
              w2,  l2, -h; ...
               w,   l,  h];
    V = V*R0' + ones(length(V),1)*t0';
    
    % Faces
    F = [1 2 4 3; ... %-X
         5 6 8 7; ... %+X
         1 2 6 5; ... %-Y
         3 4 8 7; ... %+Y
         1 3 7 5; ... %-Z
         2 4 8 6];    %+Z
    
    FV.Vertices = V;
    FV.Faces = F;
    handle.bodies(1) = patch(FV, 'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.R = eye(3);
    handle.t = t0;
    handle.labels = {'sides'};
end