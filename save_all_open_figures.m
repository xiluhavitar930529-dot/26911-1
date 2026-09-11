function save_all_open_figures(save_dir)
%SAVE_ALL_OPEN_FIGURES  保存当前打开的所有图窗为FIG和PNG

if nargin < 1 || isempty(save_dir)
    return;
end

if exist(save_dir, 'dir') ~= 7
    mkdir(save_dir);
end

figs = findall(0, 'Type', 'figure');
if isempty(figs)
    return;
end

for i = 1:numel(figs)
    fig = figs(i);
    name = get(fig, 'Name');
    if isempty(name)
        name = sprintf('figure_%02d', i);
    end
    safe_name = regexprep(name, '[\\/:*?"<>|]', '_');
    base_path = fullfile(save_dir, sprintf('%02d_%s', i, safe_name));
    try
        savefig(fig, [base_path, '.fig']);
    catch
        warning('保存FIG失败: %s.fig', base_path);
    end
    try
        saveas(fig, [base_path, '.png']);
    catch
        warning('保存PNG失败: %s.png', base_path);
    end
end

fprintf('图像已保存到: %s\n', save_dir);
end
