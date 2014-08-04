function root = rigidbodyviz_setup()
    [root,name,ext] = fileparts(mfilename('fullpath'));
    addpath(root);
    addpath(fullfile(root, 'src'));
    addpath(fullfile(root, 'sample'));
end