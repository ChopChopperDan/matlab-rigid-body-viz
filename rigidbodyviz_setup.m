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
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', 'sample'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', 'src'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', ...
                                                'src', 'create'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', ...
                                                'src', 'def'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', ...
                                                'src', 'update'));
            rmpath(fullfile(root_old, 'matlab-rigid-body-viz', ...
                                                'src', 'util'));
        end
    end
        

    addpath(root);
    addpath(fullfile(root, 'sample'));
    addpath(fullfile(root, 'src'));
    addpath(fullfile(root, 'src', 'create'));
    addpath(fullfile(root, 'src', 'def'));
    addpath(fullfile(root, 'src', 'update'));
    addpath(fullfile(root, 'src', 'util'));
end