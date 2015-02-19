function handle = createTable(R0, t0, param, varargin)
    % 
    % handle = createTable(R0, t0, param, ...)
    %
    % R0 is orientation of the table
    % t0 is the bottom center of the table
    % param is struct containing parameters for different parts
    %       surface_param *cuboid parameterization
    %           ---> width
    %           ---> length
    %           ---> height
    %       leg_param     *cuboid parameterization
    %           ---> width
    %           ---> length
    %           ---> height
    % 
    % Additional parameters include:
    %       'Color':   default: [0.8235;0.6627;0.1765]
    %
    % depends on the following drawing package files:
    %       createCuboid.m
    %       attachPrefix.m
    %
    % returns handle to drawing structure
    %
    % see also CREATECUBOID
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var'); c = [0.8235;0.6627;0.1765]; end
    
    surf_param = param.surface_param;    
    leg_param = param.leg_param;
    
    table_props = {'FaceColor', c, ...
                    'EdgeColor',[0;0;0]};
    
    surface_t0 = t0 + R0*[0;0;leg_param.height + surf_param.height/2];
    leg1_t0 = t0 + R0*[-(surf_param.width - leg_param.width)/2; ...
                        -(surf_param.length - leg_param.width)/2; ...
                        leg_param.height/2];
    leg2_t0 = t0 + R0*[-(surf_param.width - leg_param.width)/2; ...
                        (surf_param.length - leg_param.width)/2; ...
                        leg_param.height/2];
    leg3_t0 = t0 + R0*[(surf_param.width - leg_param.width)/2; ...
                        -(surf_param.length - leg_param.width)/2; ...
                        leg_param.height/2];
    leg4_t0 = t0 + R0*[(surf_param.width - leg_param.width)/2; ...
                        (surf_param.length - leg_param.width)/2; ...
                        leg_param.height/2];
    
    surface = createCuboid(R0, surface_t0, surf_param, table_props{:});
    leg1 = createCuboid(R0, leg1_t0, leg_param, table_props{:});
    leg2 = createCuboid(R0, leg2_t0, leg_param, table_props{:});
    leg3 = createCuboid(R0, leg3_t0, leg_param, table_props{:});
    leg4 = createCuboid(R0, leg4_t0, leg_param, table_props{:});
    
    handle.bodies = [surface.bodies ...
                        leg1.bodies leg2.bodies ...
                        leg3.bodies leg4.bodies];
    handle.labels = [attachPrefix('surface_', surface.labels) ...
                        attachPrefix('leg1_', leg1.labels) ...
                        attachPrefix('leg2_', leg2.labels) ...
                        attachPrefix('leg3_', leg3.labels) ...
                        attachPrefix('leg4_', leg4.labels)];
    handle.R = eye(3);
    handle.t = t0;
end
    