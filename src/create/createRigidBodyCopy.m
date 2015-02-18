function dst_handle = createRigidBodyCopy(src_handle)
    %
    % dst_handle = createRigidBodyCopy(src_handle)
    %
    % src_handle is drawing structure from which the data is coming from
    %
    % returns handle to new drawing structure which is placed in the same
    %   pose as the source object
    
    % these terms can be directly copied
    dst_handle.R = src_handle.R;
    dst_handle.t = src_handle.t;
    dst_handle.labels = src_handle.labels;
    
    nb = length(src_handle.bodies);
    dst_handle.bodies = zeros(1,nb);
    
    data = {'XData','YData','ZData'};
    props = {'FaceColor', 'FaceAlpha','EdgeColor','EdgeAlpha','LineWidth'};
    
    for i=1:nb
        XYZ = get(src_handle.bodies(i),data);
        src_props = get(src_handle.bodies(i),props);
        dst_props = reshape([props;src_props],[1 10]);
        dst_handle.bodies(i) = patch(XYZ{1},XYZ{2},XYZ{3},1);
        set(dst_handle.bodies(i),dst_props{:});
    end
end