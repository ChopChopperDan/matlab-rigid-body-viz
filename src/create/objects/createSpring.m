function handle = createSpring(R0, t0, param, varargin)
    %
    % handle = createSpring(R0, t0, param, ...)
    %
    % R0 is the 3x3 orientation matrix of the spring
    % t0 is the 3x1 position of the bottom of the spring
    % param is struct containing fields
    %       windings 
    %       wire_radius
    %       coil_radius
    %       length
    %
    % Additional parameters include:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % returns handle to drawing structure
    
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
    if isfield(param, 'windings'),  nw = param.windings;
    else  invalid = true;
    end
    if isfield(param, 'wire_radius'),  wr = param.wire_radius;
    else  invalid = true;
    end
    if isfield(param, 'coil_radius'),  cr = param.coil_radius;
    else  invalid = true;
    end
    if isfield(param, 'length'),  l = param.length;
    else  invalid = true;
    end
    if invalid
        disp('Valid parameterizations are:');
        disp('    windings, wire_radius, coil_radius, length');
        error('Invalid Parameterization for spring');
    end
    
    % Vertices
    n = 10;
    theta = 2*pi*(0:1/n:nw);
    % spiral path through center of spring
    spiral = [cr*cos(theta') cr*sin(theta') l*(0:1/(nw*n):1)'];
    
    V = zeros(n*n*nw, 3);
    phi = 2*pi*(0:1/n:(n-1)/n);
    outer_pts = wr*[cos(phi') sin(phi') zeros(size(phi'))];
    for i=1:n*nw+1
        h = [-cr*sin(theta(i)); cr*cos(theta(i)); l/(nw*n)];
        h = h / norm(h);
        V((i-1)*n+1:i*n,:) = ones(length(phi),1)*spiral(i,:) + ...
                        outer_pts*rot(hat([0;0;1])*h, acos(h(3)))';
    end
    V = V*R0' + ones(length(V),1)*t0';
    
    % Faces
    % side faces
    F1 = zeros(n*n*nw,4);
    for i=1:nw
        for j=1:n
            for k=1:n
                idx = (i-1)*n*n + (j-1)*n;
                F1(idx+k,:) = [idx+k idx+mod(k,n)+1 ...
                                idx+n+mod(k,n)+1 idx+n+k];
            end
        end
    end
    % end caps
    F2 = [1:n; n*n*nw+(1:n)];
    
    % generate handle
    handle.bodies(1) = patch(struct('Faces',F1,'Vertices',V), ...
                                    'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.bodies(2) = patch(struct('Faces',F2,'Vertices',V), ...
                                    'FaceColor',fc, ...
                                    'FaceAlpha',fa, ...
                                    'LineWidth',lw, ...
                                    'EdgeColor',ec, ...
                                    'EdgeAlpha',ea);
    handle.R = eye(3);
    handle.t = [0;0;0];
    handle.A = eye(3);
    handle.labels = {'sides', 'caps'};
end