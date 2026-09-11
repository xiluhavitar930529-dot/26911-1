function fpath = resolve_data_path(data_dir, filename)
%RESOLVE_DATA_PATH  将数据根目录和文件名组合为可用路径，兼容绝对路径

if isempty(filename)
    fpath = filename;
    return;
end

if is_absolute_path(filename)
    fpath = filename;
    return;
end

if isempty(data_dir)
    fpath = filename;
else
    fpath = fullfile(data_dir, filename);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function tf = is_absolute_path(path_str)
path_str = char(path_str);
tf = false;
if numel(path_str) >= 2 && path_str(2) == ':'
    tf = true;
elseif ~isempty(path_str) && (path_str(1) == filesep || path_str(1) == '/' || path_str(1) == '\')
    tf = true;
end
end
