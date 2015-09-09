function handle = createShelves(R0, t0, param, varargin)
    % CREATESHELVES
    %
    % handle = createShelves(R0, t0, param, ...)
    %
    % R0 is orientation of the shelves
    % t0 is the bottom center of the shelves
    % param is struct containing parameters
    %       shelf_param     *cuboid parameterization
    %           -> width
    %           -> length
    %           -> height
    %       board_width     width of boards making each shelf
    %       n_shelves       number of shelves to create
    %
    % Additional properties include
    %       'Color':   default: [0.8235;0.6627;0.1765]
    %
    % returns handle to drawing structure
    %
    % see also CREATECUBOID
    
    flags = {'Color'};
    defaults = {[0.8235; 0.6627; 0.1765]};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    c = opt_values{1};
    
    total_height = param.n_shelves*param.shelf_param.height + ...
                    (param.n_shelves + 1)*param.board_width;
    
    board_param = struct('width', param.shelf_param.width, ...
                     'length', param.shelf_param.length, ...
                     'height', param.board_width);
    
    side_param = struct('width', param.board_width, ...
                    'length', param.shelf_param.length, ...
                    'height', total_height);
    back_param = struct('width', param.shelf_param.width + ...
                                    2 * param.board_width, ...
                        'length', param.board_width, ...
                        'height', total_height);
    
    % create sides and back to shelf structure
    t_left_side = t0 + 1/2*R0*[-(board_param.width + side_param.width); ...
                       0; side_param.height];
    t_right_side = t0 + 1/2*R0*[(board_param.width + side_param.width); ...
                       0; side_param.height];
    t_back = t0 + 1/2*R0*[0;board_param.length + back_param.length; ...
                                back_param.height];
    left_side = createCuboid(R0, t_left_side,side_param,'FaceColor',c);
    left_side.labels = attachPrefix('left_side_', left_side.labels);
    right_side = createCuboid(R0, t_right_side,side_param,'FaceColor',c);
    right_side.labels = attachPrefix('right_side_', right_side.labels);
    back = createCuboid(R0, t_back,back_param,'FaceColor',c);
    back.labels = attachPrefix('back_', back.labels);
    
    handle = combineRigidBodies(left_side, right_side, back);

    % Add shelves to structure
    for n = 1:(param.n_shelves + 1)
        t_n = t0 + R0*[0;0;param.board_width/2 + ...
                    (n-1)*(param.shelf_param.height + param.board_width)];
        side_n = createCuboid(R0,t_n,board_param,'FaceColor',c);
        side_n.labels = attachPrefix(['shelf' num2str(n) '_'],side_n.labels);
        handle = combineRigidBodies(handle, side_n);
    end
end
        