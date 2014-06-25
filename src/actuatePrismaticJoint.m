function handle = actuatePrismaticJoint(dx, h, handle)
    %
    % handle = actuatePrismaticJoint(dx, h, handle)
    %
    % dx is displacement of the prismatic joint (must be >= 0)
    % h is the actuator's axis
    % handle is drawing structure for prismatic joint
    %
    % returns update structure

    idx = -1;
    for k = 1:length(handle.labels)
        if ~isempty(strfind(handle.labels{k},'slider_'))
            idx = k;
            break;
        end
    end
    if idx == -1; error('Prismatic Joint not found'); end
    
    data = {'XData';'YData';'ZData'};
    top_indeces = [1 2 5 6 9 12 13 16 17:20];
    bottom_indeces = [3 4 7 8 10 11 14 15 21:24];
    z0 = [0;0;1];
    
    if abs(z0'*h)==1
        Rj = eye(3);
    else
        phi = asin(norm(hat(z0)*h));
        if z0'*h < 0; phi = -phi; end
        Rj = rot(hat(z0)*h,phi);
    end
    
    XYZ = get(handle.bodies(idx),data);
    s = size(XYZ{1}); n = s(1)*s(2);
    XYZ_b = (handle.R*Rj)'*([XYZ{1}(:)';XYZ{2}(:)';XYZ{3}(:)'] - ...
                handle.t*ones(1,n));
    if dx < 0; dx = 0; end;
    XYZ_b(3,top_indeces) = 0;
    XYZ_b(3,bottom_indeces) = -dx;
    XYZ = handle.R*Rj*XYZ_b + handle.t*ones(1,n);
    set(handle.bodies(idx),'XData',reshape(XYZ(1,:),s), ...
                         'YData', reshape(XYZ(2,:),s), ...
                         'ZData', reshape(XYZ(3,:),s));
end