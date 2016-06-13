function empty_vis = defineEmptyRobotVis(n)
    % DEFINEEMPTYROBOTVIS
    %
    % empty_vis = defineEmptyRobotVis()
    % empty_vis = defineEmptyRobotVis(n)
    %
    % Defines an empty structure template for defining visualizations 
    % for a single n-dof robot in the matlab-rigid-body-viz toolbox.  
    % If n is undefined, assumes n=1
    %
    % see also CREATEROBOT DEFINEEMPTYROBOT
    
    if nargin == 0, n = 1; end
    if n == 0, empty_vis = []; return; end
    
    empty_vis = struct('joints', [], ...
                       'links', [], ...
                       'frame', [], ...
                       'peripherals', []);
    empty_vis.joints = struct('param',cell(1,n), ...
                                'props',cell(1,n));
    empty_vis.links = struct('handle',cell(1,n+1), ...
                                'R',cell(1,n+1), ...
                                't',cell(1,n+1), ...
                                'param',cell(1,n+1), ...
                                'props',cell(1,n+1));
    empty_vis.frame = struct('scale',[],'width',[]);
    
end
    