function platform = load_platform_txt(filepath, cfg)
%LOAD_PLATFORM_TXT  加载机载平台经纬高TXT文件
%
%  平台文件每行格式: time, lat, lon, alt (逗号分隔)
%  时间可以是 hh:mm:ss.sss 或秒数
%
%  Output:
%    platform : 结构体
%      .t_sec      [N×1] 时间戳(秒)
%      .lat_deg    [N×1] 纬度(度)
%      .lon_deg    [N×1] 经度(度)
%      .alt_m      [N×1] 高度(米)
%      .n_rows      数据行数
%      .source      来源文件名

c = cfg.platform;
max_extrapolation_s = 0.1;
if isfield(cfg, 'platform_max_extrapolation_s') && ...
        ~isempty(cfg.platform_max_extrapolation_s)
    max_extrapolation_s = cfg.platform_max_extrapolation_s;
end
if ~isscalar(max_extrapolation_s) || ~isfinite(max_extrapolation_s) || ...
        max_extrapolation_s < 0
    error('load_platform_txt:InvalidExtrapolationLimit', ...
        'cfg.platform_max_extrapolation_s must be a finite nonnegative scalar.');
end

switch lower(c.angle_unit)
    case 'deg',  ang_scale = 1;
    case 'rad',  ang_scale = 180/pi;
    case 'mrad', ang_scale = 180/(pi*1000);
    otherwise, error('未知角度单位: %s', c.angle_unit);
end

fid = fopen(filepath, 'r');
if fid < 0, error('无法打开平台文件: %s', filepath); end

%% ── 第一遍：统计数据行数 ────────────────────────────────────────────────
total_rows = 0;
while ~feof(fid)
    line = fgetl(fid);
    if ischar(line) && ~isempty(strtrim(line))
        parts = split_line_auto(strtrim(line), cfg);
        if is_platform_data_row(parts, c)
            t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
            if within_time_range(t_row, cfg)
                total_rows = total_rows + 1;
            end
        end
    end
end
fclose(fid);

[first_read_row, last_read_row, n_read, pct_start, pct_end] = read_percent_row_window(total_rows, cfg);
fprintf('[平台解析] %s: 总行=%d, 读取=%d (%.1f%%~%.1f%%, 行%d~%d)\n', ...
    filepath, total_rows, n_read, pct_start, pct_end, first_read_row, last_read_row);

%% ── 第二遍：解析数据 ────────────────────────────────────────────────────
fid = fopen(filepath, 'r');

t_sec   = zeros(max(1, n_read), 1);
lat_deg = zeros(max(1, n_read), 1);
lon_deg = zeros(max(1, n_read), 1);
alt_m   = zeros(max(1, n_read), 1);
n_rows  = 0;
data_row_count = 0;
has_header = false;
seen_first_nonempty = false;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line), continue; end
    seen_first_nonempty = true;

    parts = split_line_auto(line, cfg);
    need_cols = max([c.time_col, c.lat_col, c.lon_col, c.alt_col]);
    if numel(parts) < need_cols, continue; end

    % 检查是否是表头行
    if n_rows == 0 && seen_first_nonempty
        if ~is_platform_data_row(parts, c)
            has_header = true;
            continue;
        end
    end

    % 解析时间
    t_row = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
    if isnan(t_row), continue; end

    % 解析经纬高
    lat_v = str2double(strtrim(parts{c.lat_col}));
    lon_v = str2double(strtrim(parts{c.lon_col}));
    alt_v = str2double(strtrim(parts{c.alt_col}));

    if any(isnan([lat_v, lon_v, alt_v])), continue; end

    if ~within_time_range(t_row, cfg)
        continue;
    end
    data_row_count = data_row_count + 1;
    if data_row_count < first_read_row, continue; end
    if data_row_count > last_read_row, break; end

    n_rows = n_rows + 1;
    t_sec(n_rows)   = t_row;
    lat_deg(n_rows) = lat_v * ang_scale;
    lon_deg(n_rows) = lon_v * ang_scale;
    alt_m(n_rows)   = alt_v;
end
fclose(fid);

%% ── 截取 ──────────────────────────────────────────────────────────────
t_sec   = t_sec(1:n_rows);
lat_deg = lat_deg(1:n_rows);
lon_deg = lon_deg(1:n_rows);
alt_m   = alt_m(1:n_rows);

% 按时间排序
[t_sec, order] = sort(t_sec);
lat_deg = lat_deg(order);
lon_deg = lon_deg(order);
alt_m   = alt_m(order);

if isempty(t_sec)
    error('平台文件无有效数据行: %s', filepath);
end

% 合并重复时间戳，避免 interp1 遇到非唯一采样点
[t_unique, ~, ic] = unique(t_sec);
if numel(t_unique) < numel(t_sec)
    lat_deg = accumarray(ic, lat_deg, [], @mean);
    lon_deg = accumarray(ic, lon_deg, [], @mean);
    alt_m   = accumarray(ic, alt_m,   [], @mean);
    t_sec   = t_unique;
    n_rows  = numel(t_sec);
end

[~, fname, ext] = fileparts(filepath);
platform = struct();
platform.t_sec   = t_sec;
platform.lat_deg = lat_deg;
platform.lon_deg = lon_deg;
platform.alt_m   = alt_m;
platform.n_rows  = n_rows;
platform.source  = [fname, ext];

fprintf('[平台加载] %s: 表头=%d, 总行=%d, 读取=%d, 有效=%d\n', ...
    filepath, has_header, total_rows, n_read, n_rows);
if n_rows > 0
    if n_rows > 1
        dt_median = median(diff(t_sec));
        fprintf('  时间: %.3f ~ %.3f s, 采样间隔中位数=%.4f s (%.1f Hz)\n', ...
            min(t_sec), max(t_sec), dt_median, 1/dt_median);
    else
        fprintf('  时间: %.3f s, 仅1条平台记录，插值将使用常值\n', t_sec(1));
    end
    fprintf('  纬度: %.4f~%.4f deg, 经度: %.4f~%.4f deg, 高度: %.0f~%.0f m\n', ...
        min(lat_deg), max(lat_deg), min(lon_deg), max(lon_deg), min(alt_m), max(alt_m));
end

%% ── 构建插值函数句柄 ──────────────────────────────────────────────────
% 端点外只允许很短的线性外推；超限直接报错，避免静默钳位制造假几何。
if n_rows > 1
    max_extrapolation_s = min(max_extrapolation_s, median(diff(t_sec)));
else
    max_extrapolation_s = 0;
end
platform.max_extrapolation_s = max_extrapolation_s;
platform.interp_lat = @(t) interp1_bounded(t_sec, lat_deg, t, max_extrapolation_s);
platform.interp_lon = @(t) interp1_bounded(t_sec, lon_deg, t, max_extrapolation_s);
platform.interp_alt = @(t) interp1_bounded(t_sec, alt_m, t, max_extrapolation_s);

end

%% ═══════════════════════════════════════════════════════════════════════════
function t_sec = parse_time_str(t_str, fmt)
t_str = strtrim(char(t_str));
t_str = strrep(t_str, char(65279), '');
if isempty(t_str), t_sec = NaN; return; end
t_num = str2double(t_str);
if ~isnan(t_num), t_sec = t_num; return; end
switch lower(fmt)
    case 'hms'
        parts = strsplit(t_str, ':');
        if numel(parts) < 3, t_sec = NaN; return; end
        h = str2double(parts{1}); m = str2double(parts{2}); s = str2double(parts{3});
        if any(isnan([h, m, s])), t_sec = NaN; return; end
        t_sec = h * 3600 + m * 60 + s;
    otherwise
        t_sec = str2double(t_str);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function yi = interp1_bounded(x, y, xi, limit_s)
if isempty(xi)
    yi = zeros(size(xi));
    return;
end
if any(~isfinite(xi(:)))
    error('load_platform_txt:InvalidQueryTime', ...
        'Platform query times must be finite.');
end
if any(xi(:) < x(1) - limit_s - 1e-9 | ...
        xi(:) > x(end) + limit_s + 1e-9)
    error('load_platform_txt:PlatformTimeCoverage', ...
        ['平台时间范围[%.6f, %.6f]未覆盖查询范围[%.6f, %.6f]，' ...
         '已超过允许的端点线性外推上限 %.6f s。'], ...
        x(1), x(end), min(xi(:)), max(xi(:)), limit_s);
end
if numel(x) == 1
    yi = y(1) * ones(size(xi));
    return;
end
yi = reshape(interp1(x, y, xi(:), 'linear', 'extrap'), size(xi));
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
function tf = is_platform_data_row(parts, c)
need_cols = max([c.time_col, c.lat_col, c.lon_col, c.alt_col]);
if numel(parts) < need_cols
    tf = false;
    return;
end
t_val = parse_time_str(strtrim(parts{c.time_col}), c.time_format);
lat_v = str2double(strtrim(parts{c.lat_col}));
lon_v = str2double(strtrim(parts{c.lon_col}));
alt_v = str2double(strtrim(parts{c.alt_col}));
tf = ~isnan(t_val) && ~any(isnan([lat_v, lon_v, alt_v]));
end
