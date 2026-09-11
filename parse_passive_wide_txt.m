function passive_data = parse_passive_wide_txt(filepath, cfg)
%PARSE_PASSIVE_WIDE_TXT  解析被动雷达宽表TXT文件（仅方位俯仰，无距离）
%
%  Inputs:
%    filepath : TXT文件路径
%    cfg      : config_fusion() 返回的配置结构
%
%  Output:
%    passive_data : 结构体
%      .t_sec    [N×1] 时间戳(秒)
%      .az_deg   [N×1] 方位角(度)
%      .el_deg   [N×1] 俯仰角(度)
%      .target_id [N×1] 目标编号
%      .source   字符串  来源文件名

c = cfg.passive;

%% ── 角度单位缩放 ──────────────────────────────────────────────────────
switch lower(c.angle_unit)
    case 'deg',  ang_scale = 1;
    case 'rad',  ang_scale = 180/pi;
    case 'mrad', ang_scale = 180/(pi*1000);
    otherwise, error('未知角度单位: %s', c.angle_unit);
end

%% ── 第一步：统计数据行数 ────────────────────────────────────────────────
fid = fopen(filepath, 'r');
if fid < 0, error('无法打开文件: %s', filepath); end

total_data_rows = 0;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line), continue; end
    parts = split_line_auto(line, cfg);
    if is_passive_data_row(parts, c)
        t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
        if within_time_range(t_row, cfg)
            total_data_rows = total_data_rows + 1;
        end
    end
end
fclose(fid);

%% ── 计算读取行范围 ────────────────────────────────────────────────────
[first_read_row, last_read_row, n_read, pct_start, pct_end] = read_percent_row_window(total_data_rows, cfg);

fprintf('[被动解析] %s: 总行=%d, 读取=%d (%.1f%%~%.1f%%, 行%d~%d)\n', ...
    filepath, total_data_rows, n_read, pct_start, pct_end, first_read_row, last_read_row);

%% ── 第二步：解析数据行 ────────────────────────────────────────────────
fid = fopen(filepath, 'r');
if fid < 0, error('无法打开文件: %s', filepath); end

max_targets = get_max_targets_per_row(cfg);
max_meas = max(1, n_read * max_targets);
t_sec     = zeros(max_meas, 1);
az_deg    = zeros(max_meas, 1);
el_deg    = zeros(max_meas, 1);
target_id = nan(max_meas, 1);
n_meas = 0;

row_count = 0;
n_skip_invalid = 0;
n_skip_missing = 0;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line), continue; end

    parts = split_line_auto(line, cfg);
    if ~is_passive_data_row(parts, c)
        continue;
    end

    t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
    if isnan(t_row), continue; end

    if ~within_time_range(t_row, cfg)
        continue;
    end
    row_count = row_count + 1;
    if row_count < first_read_row, continue; end
    if row_count > last_read_row, break; end

    n_targets = str2double(strtrim(parts{c.count_col}));
    if isnan(n_targets) || n_targets <= 0, continue; end
    n_targets = min(floor(n_targets), max_targets);

    for g = 1:n_targets
        off = (g - 1) * c.stride;

        col_tid   = c.target_id_col + off;
        col_valid = c.valid_cols + off;   % [az_valid, el_valid]
        col_az    = c.az_col + off;
        col_el    = c.el_col + off;

        max_col = max([col_tid, col_valid, col_az, col_el]);
        if max_col > numel(parts)
            n_skip_missing = n_skip_missing + 1;
            continue;
        end

        % 检查有效标识（全部为1才有效）
        valid_all = true;
        for vi = 1:numel(col_valid)
            vf = str2double(strtrim(parts{col_valid(vi)}));
            if isnan(vf) || vf < 0.5
                valid_all = false;
                break;
            end
        end
        if ~valid_all
            n_skip_invalid = n_skip_invalid + 1;
            continue;
        end

        % 读取角度值
        az_v = str2double(strtrim(parts{col_az}));
        el_v = str2double(strtrim(parts{col_el}));

        if any(isnan([az_v, el_v]))
            n_skip_missing = n_skip_missing + 1;
            continue;
        end

        tid_v = str2double(strtrim(parts{col_tid}));
        if isnan(tid_v), tid_v = NaN; end

        n_meas = n_meas + 1;
        if n_meas > numel(t_sec)
            [t_sec, az_deg, el_deg, target_id] = grow_arrays( ...
                t_sec, az_deg, el_deg, target_id);
        end
        t_sec(n_meas)     = t_row;
        az_deg(n_meas)    = az_v * ang_scale;
        el_deg(n_meas)    = el_v * ang_scale;
        target_id(n_meas) = tid_v;
    end
end
fclose(fid);

%% ── 截取 ──────────────────────────────────────────────────────────────
t_sec     = t_sec(1:n_meas);
az_deg    = az_deg(1:n_meas);
el_deg    = el_deg(1:n_meas);
target_id = target_id(1:n_meas);

[t_sec, order] = sort(t_sec);
az_deg    = az_deg(order);
el_deg    = el_deg(order);
target_id = target_id(order);

[~, fname, ext] = fileparts(filepath);
passive_data = struct();
passive_data.t_sec     = t_sec;
passive_data.az_deg    = az_deg;
passive_data.el_deg    = el_deg;
passive_data.target_id = target_id;
passive_data.source    = [fname, ext];
passive_data.n_meas    = n_meas;

fprintf('  有效量测=%d, 跳过(无效=%d,缺失=%d)\n', ...
    n_meas, n_skip_invalid, n_skip_missing);
if n_meas > 0
    fprintf('  时间: %.3f ~ %.3f s, az: %.1f~%.1f deg, el: %.1f~%.1f deg\n', ...
        min(t_sec), max(t_sec), min(az_deg), max(az_deg), min(el_deg), max(el_deg));
end

end

%% ═══════════════════════════════════════════════════════════════════════════
function t_sec = parse_time_str(t_str, fmt)
t_str = strtrim(char(t_str));
t_str = strrep(t_str, char(65279), '');
if isempty(t_str)
    t_sec = NaN; return;
end
t_num = str2double(t_str);
if ~isnan(t_num)
    t_sec = t_num; return;
end
switch lower(fmt)
    case 'hms'
        t_sec = hms_to_seconds(t_str);
    otherwise
        t_sec = str2double(t_str);
end
end

function sec = hms_to_seconds(t_str)
parts = strsplit(strtrim(t_str), ':');
if numel(parts) < 3, sec = NaN; return; end
h = str2double(parts{1}); m = str2double(parts{2}); s = str2double(parts{3});
if any(isnan([h, m, s])), sec = NaN; return; end
sec = h * 3600 + m * 60 + s;
end

%% ═══════════════════════════════════════════════════════════════════════════
function parts = split_line_auto(line, cfg)
delimiter = 'auto';
if isfield(cfg, 'delimiter') && ~isempty(cfg.delimiter)
    delimiter = cfg.delimiter;
end

if ischar(delimiter) && strcmpi(delimiter, 'auto')
    if ~isempty(strfind(line, ','))
        parts = strsplit(line, ',', 'CollapseDelimiters', false);
    elseif ~isempty(strfind(line, sprintf('\t')))
        parts = strsplit(line, sprintf('\t'), 'CollapseDelimiters', false);
    elseif ~isempty(strfind(line, ';'))
        parts = strsplit(line, ';', 'CollapseDelimiters', false);
    else
        parts = regexp(strtrim(line), '\s+', 'split');
    end
else
    parts = strsplit(line, delimiter, 'CollapseDelimiters', false);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function tf = is_passive_data_row(parts, c)
need_cols = max(c.time_col, c.count_col);
if numel(parts) < need_cols
    tf = false;
    return;
end
t_val = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
n_val = str2double(strtrim(parts{c.count_col}));
tf = ~isnan(t_val) && ~isnan(n_val);
end

%% ═══════════════════════════════════════════════════════════════════════════
function tf = within_time_range(t_sec, cfg)
tf = true;
if isfield(cfg, 'time_range_s') && ~isempty(cfg.time_range_s)
    tr = cfg.time_range_s;
    if numel(tr) == 2
        tf = (t_sec >= tr(1)) && (t_sec <= tr(2));
    end
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function max_targets = get_max_targets_per_row(cfg)
if isfield(cfg, 'max_targets_per_row') && isfinite(cfg.max_targets_per_row)
    max_targets = max(1, floor(cfg.max_targets_per_row));
else
    max_targets = 64;
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function [t_sec, az_deg, el_deg, target_id] = grow_arrays( ...
        t_sec, az_deg, el_deg, target_id)
grow_by = max(1024, numel(t_sec));
t_sec     = [t_sec;     zeros(grow_by, 1)];
az_deg    = [az_deg;    zeros(grow_by, 1)];
el_deg    = [el_deg;    zeros(grow_by, 1)];
target_id = [target_id; nan(grow_by, 1)];
end
