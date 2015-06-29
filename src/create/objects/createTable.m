function handle = createTable(R0, t0, param, varargin)
    % 
    % handle = createTable(R0, t0, param, ...)
    %
    % R0 is orientation of the table
    % t0 is the bottom center of the table
    % param is struct containing parameters for different parts
    %       surface_param   *cuboid parameterization
    %           ---> width
    %           ---> length
    %           ---> height
    %       leg_param       *cuboid parameterization
    %           ---> width
    %           ---> length
    %           ---> height
    % 
    % Additional parameters include:
    %       'Color':   default: [0.8235;0.6627;0.1765]
    %
    % returns handle to drawing structure
    %
    % see also CREATECUBOID
    
    flags = {'Color'};
    defaults = {[0.8235;0.6627;0.1765]};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    c = opt_values{1};
    
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
    
    surface.labels = attachPrefix('surface_', surface.labels);
    leg1.labels = attachPrefix('leg1_', leg1.labels);
    leg2.labels = attachPrefix('leg2_', leg2.labels);
    leg3.labels = attachPrefix('leg3_', leg3.labels);
    leg4.labels = attachPrefix('leg4_', leg4.labels);
    
    handle = combineRigidBodies(surface, leg1, leg2, leg3, leg4);
end
    