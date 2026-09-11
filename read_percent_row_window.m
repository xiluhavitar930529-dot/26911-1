function [first_row, last_row, n_read, pct_start, pct_end] = read_percent_row_window(total_rows, cfg)
%READ_PERCENT_ROW_WINDOW  Convert configured read percentage window to row indexes.
%   Percentages are applied after time_range_s filtering and before max_rows.
%   Legacy cfg.read_percent is kept as the default end percentage, so
%   read_percent=10 still means 0%~10%.

if nargin < 1 || isempty(total_rows) || ~isfinite(total_rows) || total_rows < 0
    total_rows = 0;
end
total_rows = floor(total_rows);

if isfield(cfg, 'read_start_percent') && ~isempty(cfg.read_start_percent)
    pct_start = cfg.read_start_percent;
else
    pct_start = 0;
end

if isfield(cfg, 'read_end_percent') && ~isempty(cfg.read_end_percent)
    pct_end = cfg.read_end_percent;
elseif isfield(cfg, 'read_percent') && ~isempty(cfg.read_percent)
    pct_end = cfg.read_percent;
else
    pct_end = 100;
end

pct_start = min(max(pct_start, 0), 100);
pct_end = min(max(pct_end, 0), 100);

if total_rows == 0 || pct_end <= pct_start
    first_row = 1;
    last_row = 0;
    n_read = 0;
    return;
end

first_row = floor(total_rows * pct_start / 100) + 1;
last_row = ceil(total_rows * pct_end / 100);
first_row = min(max(first_row, 1), total_rows + 1);
last_row = min(max(last_row, 0), total_rows);

if last_row < first_row
    n_read = 0;
    return;
end

if isfield(cfg, 'max_rows') && ~isempty(cfg.max_rows) && isfinite(cfg.max_rows)
    max_rows = max(0, floor(cfg.max_rows));
    last_row = min(last_row, first_row + max_rows - 1);
end

n_read = max(0, last_row - first_row + 1);
end
