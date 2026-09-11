function [data, status] = parse_radar_file_cached(filepath, kind, cfg, cache_only)
%PARSE_RADAR_FILE_CACHED Parse one radar file with validated disk caching.

if nargin < 4, cache_only = false; end

kind = normalize_kind(kind);
status = struct('enabled', false, 'hit', false, 'written', false, ...
    'cache_file', '', 'reason', 'disabled');
cache_enabled = isfield(cfg, 'parse_cache_enabled') && cfg.parse_cache_enabled;
if ~cache_enabled
    if cache_only, data = []; else, data = parse_source(filepath, kind, cfg); end
    return;
end

status.enabled = true;
status.reason = 'miss';
try
    signature = build_signature(filepath, kind, cfg);
    cache_file = cache_path(signature.source_path, kind, cfg.parse_cache_dir);
    status.cache_file = cache_file;
catch ME
    status.reason = ['signature_error:', ME.identifier];
    if cache_only, data = []; else, data = parse_source(filepath, kind, cfg); end
    return;
end

if exist(cache_file, 'file') == 2
    try
        loaded = load(cache_file, 'cache_entry');
        if isfield(loaded, 'cache_entry') && ...
                isstruct(loaded.cache_entry) && ...
                isfield(loaded.cache_entry, 'signature') && ...
                isfield(loaded.cache_entry, 'data') && ...
                isequaln(loaded.cache_entry.signature, signature) && ...
                valid_parsed_data(loaded.cache_entry.data, kind)
            data = loaded.cache_entry.data;
            status.hit = true;
            status.reason = 'hit';
            return;
        end
        status.reason = 'stale';
    catch ME
        status.reason = ['read_error:', ME.identifier];
    end
end

if cache_only
    data = [];
    return;
end

data = parse_source(filepath, kind, cfg);
if exist(cfg.parse_cache_dir, 'dir') ~= 7
    try
        mkdir(cfg.parse_cache_dir);
    catch ME
        status.reason = ['mkdir_error:', ME.identifier];
        return;
    end
end

tmp_file = [tempname(cfg.parse_cache_dir), '.mat'];
cleanup = onCleanup(@() delete_if_exists(tmp_file));
try
    cache_entry = struct('signature', signature, 'data', data);
    save(tmp_file, 'cache_entry', '-v7');
    [ok, msg] = movefile(tmp_file, cache_file, 'f');
    if ok
        status.written = true;
    else
        status.reason = ['write_error:', msg];
    end
catch ME
    status.reason = ['write_error:', ME.identifier];
end
end

function kind = normalize_kind(kind)
if isnumeric(kind) && isscalar(kind)
    if kind == 1, kind = 'active'; elseif kind == 2, kind = 'passive'; end
elseif isa(kind, 'string') && isscalar(kind)
    kind = char(kind);
end
if ~ischar(kind) || ~any(strcmp(kind, {'active', 'passive'}))
    error('parse_radar_file_cached:InvalidKind', 'kind必须是active/passive或1/2');
end
end

function data = parse_source(filepath, kind, cfg)
if strcmp(kind, 'active')
    data = parse_active_wide_txt(filepath, cfg);
else
    data = parse_passive_wide_txt(filepath, cfg);
end
end

function signature = build_signature(filepath, kind, cfg)
filepath = absolute_path(filepath);
source_info = dir(filepath);
if isempty(source_info) || source_info(1).isdir
    error('parse_radar_file_cached:MissingSource', '雷达文件不存在: %s', filepath);
end
source_info = source_info(1);
parser_name = ['parse_', kind, '_wide_txt'];
parser_file = which(parser_name);
helper_file = which('read_percent_row_window');

signature = struct();
signature.schema_version = 1;
signature.source_path = char(filepath);
signature.source_bytes = source_info.bytes;
signature.source_datenum = source_info.datenum;
signature.kind = kind;
signature.delimiter = cfg.delimiter;
signature.read_percent = cfg.read_percent;
signature.read_start_percent = cfg.read_start_percent;
signature.read_end_percent = cfg.read_end_percent;
signature.max_rows = cfg.max_rows;
signature.max_targets_per_row = cfg.max_targets_per_row;
signature.time_range_s = cfg.time_range_s;
signature.layout = cfg.(kind);
signature.parser_file = file_signature(parser_file);
signature.window_helper_file = file_signature(helper_file);
end

function value = file_signature(filepath)
info = dir(filepath);
if isempty(info)
    value = struct('path', char(filepath), 'bytes', NaN, 'datenum', NaN);
else
    value = struct('path', char(filepath), 'bytes', info(1).bytes, ...
        'datenum', info(1).datenum);
end
end

function filepath = cache_path(source_path, kind, cache_dir)
[~, base, ~] = fileparts(source_path);
safe_base = regexprep(base, '[^A-Za-z0-9_-]', '_');
if isempty(safe_base), safe_base = 'radar'; end
safe_base = safe_base(1:min(numel(safe_base), 48));
token = hash_text([kind, '|', char(source_path)]);
filepath = fullfile(cache_dir, sprintf('%s_%s_%s.mat', kind, safe_base, token));
end

function filepath = absolute_path(filepath)
filepath = char(filepath);
is_drive_path = numel(filepath) >= 2 && filepath(2) == ':';
is_rooted_path = ~isempty(filepath) && any(filepath(1) == ['/', '\']);
if ~is_drive_path && ~is_rooted_path
    filepath = fullfile(pwd, filepath);
end
filepath = strrep(filepath, '/', filesep);
end

function token = hash_text(value)
bytes = unicode2native(lower(char(value)), 'UTF-8');
h = uint64(2166136261);
mask = uint64(4294967295);
prime = uint64(16777619);
for i = 1:numel(bytes)
    h = bitxor(h, uint64(bytes(i)));
    h = bitand(h * prime, mask);
end
token = lower(dec2hex(h, 8));
end

function tf = valid_parsed_data(data, kind)
common = {'t_sec', 'az_deg', 'el_deg', 'target_id', 'source', 'n_meas'};
if strcmp(kind, 'active')
    required = [common, {'range_m', 'range_valid'}];
else
    required = common;
end
tf = isstruct(data) && isscalar(data) && all(isfield(data, required));
if ~tf || ~isscalar(data.n_meas) || ~isfinite(data.n_meas) || ...
        data.n_meas < 0 || floor(data.n_meas) ~= data.n_meas
    tf = false;
    return;
end
n = data.n_meas;
for i = 1:numel(common) - 2
    if numel(data.(common{i})) ~= n
        tf = false;
        return;
    end
end
if strcmp(kind, 'active') && ...
        (numel(data.range_m) ~= n || numel(data.range_valid) ~= n)
    tf = false;
end
end

function delete_if_exists(filepath)
if exist(filepath, 'file') == 2
    delete(filepath);
end
end
