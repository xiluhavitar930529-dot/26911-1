function active_data = parse_active_wide_txt(filepath, cfg)
%PARSE_ACTIVE_WIDE_TXT  解析主动雷达宽表TXT文件
%
%  Inputs:
%    filepath : TXT文件路径
%    cfg      : config_fusion() 返回的配置结构
%
%  Output:
%    active_data : 结构体
%      .t_sec    [N×1] 时间戳(秒)
%      .az_deg   [N×1] 地理系方位角(度)
%      .el_deg   [N×1] 地理系俯仰角(度)
%      .range_m  [N×1] 斜距(米)
%      .target_id [N×1] 目标编号
%      .source   字符串  来源文件名

c = cfg.active;

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
    if is_active_data_row(parts, c)
        t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
        if within_time_range(t_row, cfg)
            total_data_rows = total_data_rows + 1;
        end
    end
end
fclose(fid);

%% ── 计算读取行范围 ────────────────────────────────────────────────────
[first_read_row, last_read_row, n_read, pct_start, pct_end] = read_percent_row_window(total_data_rows, cfg);

fprintf('[主动解析] %s: 总行=%d, 读取=%d (%.1f%%~%.1f%%, 行%d~%d)\n', ...
    filepath, total_data_rows, n_read, pct_start, pct_end, first_read_row, last_read_row);

%% ── 第二步：解析数据行 ────────────────────────────────────────────────
fid = fopen(filepath, 'r');
if fid < 0, error('无法打开文件: %s', filepath); end

% 预分配；真实文件目标数可能超过初始估计，后续会自动扩容
max_targets = get_max_targets_per_row(cfg);
max_meas = max(1, n_read * max_targets);
t_sec    = zeros(max_meas, 1);
az_deg   = zeros(max_meas, 1);
el_deg   = zeros(max_meas, 1);
range_m  = zeros(max_meas, 1);
range_valid = false(max_meas, 1);
target_id = nan(max_meas, 1);
n_meas = 0;

row_count = 0;
n_skip_invalid = 0;
n_skip_missing = 0;
n_ae_only      = 0;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line), continue; end

    parts = split_line_auto(line, cfg);
    if ~is_active_data_row(parts, c)
        continue;
    end

    % 解析时间
    t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
    if isnan(t_row), continue; end

    if ~within_time_range(t_row, cfg)
        continue;
    end
    row_count = row_count + 1;
    if row_count < first_read_row, continue; end
    if row_count > last_read_row, break; end

    % 目标数量
    n_targets = str2double(strtrim(parts{c.count_col}));
    if isnan(n_targets) || n_targets <= 0, continue; end
    n_targets = min(floor(n_targets), max_targets);

    % 遍历每个目标块
    for g = 1:n_targets
        off = (g - 1) * c.stride;

        col_tid   = c.target_id_col + off;
        col_valid = c.valid_cols + off;   % [az_valid, el_valid, range_valid]
        col_az    = c.az_col + off;
        col_el    = c.el_col + off;
        col_range = c.range_col + off;

        max_col = max([col_tid, col_valid, col_az, col_el, col_range]);
        if max_col > numel(parts)
            n_skip_missing = n_skip_missing + 1;
            continue;
        end

        % 方位/俯仰有效即可进入二维分支；距离有效性单独保留。
        valid_flags = nan(1, numel(col_valid));
        for vi = 1:numel(col_valid)
            valid_flags(vi) = str2double(strtrim(parts{col_valid(vi)}));
        end
        ae_valid = numel(valid_flags) >= 2 && all(isfinite(valid_flags(1:2))) && ...
            all(valid_flags(1:2) >= 0.5);
        range_ok = numel(valid_flags) >= 3 && isfinite(valid_flags(3)) && ...
            valid_flags(3) >= 0.5;
        if ~ae_valid
            n_skip_invalid = n_skip_invalid + 1;
            continue;
        end

        % 读取量测值
        az_v = str2double(strtrim(parts{col_az}));
        el_v = str2double(strtrim(parts{col_el}));
        r_v  = str2double(strtrim(parts{col_range}));

        if any(isnan([az_v, el_v]))
            n_skip_missing = n_skip_missing + 1;
            continue;
        end
        range_ok = range_ok && isfinite(r_v) && r_v > 0;
        if ~range_ok
            r_v = NaN;
            n_ae_only = n_ae_only + 1;
        end

        % 读取目标编号
        tid_v = str2double(strtrim(parts{col_tid}));
        if isnan(tid_v), tid_v = NaN; end

        n_meas = n_meas + 1;
        if n_meas > numel(t_sec)
            [t_sec, az_deg, el_deg, range_m, range_valid, target_id] = grow_arrays( ...
                t_sec, az_deg, el_deg, range_m, range_valid, target_id);
        end
        t_sec(n_meas)    = t_row;
        az_deg(n_meas)   = az_v * ang_scale;
        el_deg(n_meas)   = el_v * ang_scale;
        range_m(n_meas)  = r_v;
        range_valid(n_meas) = range_ok;
        target_id(n_meas) = tid_v;
    end
end
fclose(fid);

%% ── 截取有效数据 ──────────────────────────────────────────────────────
t_sec     = t_sec(1:n_meas);
az_deg    = az_deg(1:n_meas);
el_deg    = el_deg(1:n_meas);
range_m   = range_m(1:n_meas);
range_valid = range_valid(1:n_meas);
target_id = target_id(1:n_meas);

%% ── 按时间排序 ────────────────────────────────────────────────────────
[t_sec, order] = sort(t_sec);
az_deg    = az_deg(order);
el_deg    = el_deg(order);
range_m   = range_m(order);
range_valid = range_valid(order);
target_id = target_id(order);

[~, fname, ext] = fileparts(filepath);
active_data = struct();
active_data.t_sec     = t_sec;
active_data.az_deg    = az_deg;
active_data.el_deg    = el_deg;
active_data.range_m   = range_m;
active_data.range_valid = range_valid;
active_data.target_id = target_id;
active_data.source    = [fname, ext];
active_data.n_meas    = n_meas;

fprintf('  有效AE量测=%d (完整RAE=%d, AE-only=%d), 跳过(无效=%d,缺失=%d)\n', ...
    n_meas, nnz(range_valid), n_ae_only, n_skip_invalid, n_skip_missing);
if n_meas > 0
    rv = range_m(range_valid & isfinite(range_m));
    if isempty(rv)
        fprintf('  时间: %.3f ~ %.3f s, 无有效距离量测\n', min(t_sec), max(t_sec));
    else
        fprintf('  时间: %.3f ~ %.3f s, 距离: %.0f ~ %.0f m\n', ...
            min(t_sec), max(t_sec), min(rv), max(rv));
    end
end

end

%% ═══════════════════════════════════════════════════════════════════════════
function t_sec = parse_time_str(t_str, fmt)
% 解析时间字符串/数值 → 秒数
t_str = strtrim(char(t_str));
t_str = strrep(t_str, char(65279), '');
if isempty(t_str)
    t_sec = NaN;
    return;
end

% 尝试直接转为数值（已经是秒数的情况）
t_num = str2double(t_str);
if ~isnan(t_num)
    t_sec = t_num;
    return;
end

% 尝试 hh:mm:ss.sss 格式
switch lower(fmt)
    case 'hms'
        t_sec = hms_to_seconds(t_str);
    otherwise
        t_sec = str2double(t_str);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function sec = hms_to_seconds(t_str)
% 将 hh:mm:ss.sss 字符串转为从当日零时起的秒数
parts = strsplit(strtrim(t_str), ':');
if numel(parts) < 3
    sec = NaN;
    return;
end
h = str2double(parts{1});
m = str2double(parts{2});
s = str2double(parts{3});
if any(isnan([h, m, s]))
    sec = NaN;
    return;
end
sec = h * 3600 + m * 60 + s;
end

%% ═══════════════════════════════════════════════════════════════════════════
function parts = split_line_auto(line, cfg)
% 支持逗号、制表符、分号和空白分隔；默认按CSV逗号处理
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
function tf = is_active_data_row(parts, c)
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
function [t_sec, az_deg, el_deg, range_m, range_valid, target_id] = grow_arrays(t_sec, az_deg, el_deg, range_m, range_valid, target_id)
grow_by = max(1024, numel(t_sec));
t_sec     = [t_sec;     zeros(grow_by, 1)];
az_deg    = [az_deg;    zeros(grow_by, 1)];
el_deg    = [el_deg;    zeros(grow_by, 1)];
range_m   = [range_m;   zeros(grow_by, 1)];
range_valid = [range_valid; false(grow_by, 1)];
target_id = [target_id; nan(grow_by, 1)];
end
