function root = rigidbodyviz_setup()
    %%% TODO: Should incorporate some sort of version control


    [root,name,ext] = fileparts(mfilename('fullpath'));
    % check if matlab-rigid-body-viz is already added to path
    idx = strfind(path,'matlab-rigid-body-viz');
    if ~isempty(idx)
        % if this root path has already been added, simply return
        % otherwise, remove old path
        if ~isempty(strfind(path,root))
            return;
        else
            path_str = path;
            idx0 = strfind(1:idx(1),';');
            if isempty(idx0)
                root_old = path_str(1:idx(1)-1);
            else
                root_old = path_str(idx0+1:idx(1)-1);
            end
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz'))
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', 'src'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', 'sample'));
        end
    end
        

    addpath(root);
    addpath(fullfile(root, 'src'));
    addpath(fullfile(root, 'sample'));
end