function N = calc_normals(FV)
    nf = size(FV.Faces,1);
    N = zeros(nf, 3);
    for i=1:nf
        f = FV.Faces(i, ~isnan(FV.Faces(i,:)));
        v = FV.Vertices(f,:);
        n = [0;0;0];
        for j = 1:size(v,1)-1
            n = n + hat(v(j,:)')*v(j+1,:)';
        end
        n = n + hat(v(end,:)')*v(1,:)';
        N(i,:) = n' / norm(n);
    end
end