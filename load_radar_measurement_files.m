function [active_list, passive_list, info] = load_radar_measurement_files(cfg)
%LOAD_RADAR_MEASUREMENT_FILES Parse independent radar files in parallel.

n_active = numel(cfg.active_files); n_passive = numel(cfg.passive_files);
n_tasks = n_active + n_passive;
active_list = cell(n_active, 1); passive_list = cell(n_passive, 1);
info = struct('mode', 'serial', 'n_workers', 1, 'elapsed_s', 0, ...
    'cache_enabled', false, 'cache_hits', 0, 'cache_misses', 0, ...
    'cache_writes', 0, 'cache_files', {{}});
if n_tasks == 0, return; end

paths = cell(n_tasks, 1); kinds = zeros(n_tasks, 1);
for i = 1:n_active
    paths{i} = resolve_data_path(cfg.data_dir, cfg.active_files{i});
    kinds(i) = 1;
end
for i = 1:n_passive
    q = n_active + i;
    paths{q} = resolve_data_path(cfg.data_dir, cfg.passive_files{i});
    kinds(q) = 2;
end

use_parallel = isfield(cfg, 'parallel_file_loading') && cfg.parallel_file_loading && ...
    n_tasks > 1 && license('test', 'Distrib_Computing_Toolbox') && ...
    exist('parpool', 'file') == 2;
t0 = tic;
cache_enabled = isfield(cfg, 'parse_cache_enabled') && cfg.parse_cache_enabled;
info.cache_enabled = cache_enabled;
if cache_enabled && exist(cfg.parse_cache_dir, 'dir') ~= 7
    try
        mkdir(cfg.parse_cache_dir);
    catch ME
        warning('load_radar_measurement_files:CacheDirectory', ...
            '无法创建解析缓存目录，将继续执行且不依赖缓存: %s', ME.message);
    end
end
parsed = cell(n_tasks, 1);
cache_status = cell(n_tasks, 1);
pending = 1:n_tasks;
if cache_enabled
    for q = 1:n_tasks
        [parsed{q}, cache_status{q}] = parse_radar_file_cached( ...
            paths{q}, kinds(q), cfg, true);
    end
    pending = find(~cellfun(@(s) s.hit, cache_status));
end

if isempty(pending)
    info.mode = 'cache';
    info.n_workers = 0;
elseif use_parallel && numel(pending) > 1
    try
        pool = gcp('nocreate');
        if isempty(pool)
            cluster = parcluster('local');
            job_dir = fullfile(tempdir, 'fusion_parallel_jobs');
            if exist(job_dir, 'dir') ~= 7, mkdir(job_dir); end
            cluster.JobStorageLocation = job_dir;
            requested = cfg.parallel_file_workers;
            if requested <= 0, requested = cluster.NumWorkers; end
            requested = min([requested, numel(pending), cluster.NumWorkers]);
            pool = parpool(cluster, max(1, requested));
        end
        pending_data = cell(numel(pending), 1);
        pending_status = cell(numel(pending), 1);
        pending_paths = paths(pending);
        pending_kinds = kinds(pending);
        parfor r = 1:numel(pending)
            [pending_data{r}, pending_status{r}] = ...
                parse_radar_file_cached(pending_paths{r}, pending_kinds(r), cfg);
        end
        for r = 1:numel(pending)
            q = pending(r);
            parsed{q} = pending_data{r};
            cache_status{q} = pending_status{r};
        end
        info.mode = 'parallel'; info.n_workers = pool.NumWorkers;
    catch ME
        warning('load_radar_measurement_files:ParallelFallback', ...
            '并行文件解析失败，回退串行: %s', ME.message);
        for r = 1:numel(pending)
            q = pending(r);
            [parsed{q}, cache_status{q}] = ...
                parse_radar_file_cached(paths{q}, kinds(q), cfg);
        end
        info.mode = 'serial_fallback'; info.n_workers = 1;
    end
else
    for r = 1:numel(pending)
        q = pending(r);
        [parsed{q}, cache_status{q}] = ...
            parse_radar_file_cached(paths{q}, kinds(q), cfg);
    end
end
active_list = parsed(1:n_active);
passive_list = parsed(n_active + 1:end);
info = summarize_cache(info, cache_status);
info.elapsed_s = toc(t0);
end

function info = summarize_cache(info, cache_status)
if isempty(cache_status), return; end
valid = ~cellfun(@isempty, cache_status);
cache_status = cache_status(valid);
if isempty(cache_status), return; end
info.cache_hits = sum(cellfun(@(s) s.hit, cache_status));
info.cache_writes = sum(cellfun(@(s) s.written, cache_status));
info.cache_misses = sum(cellfun(@(s) s.enabled && ~s.hit, cache_status));
info.cache_files = cellfun(@(s) s.cache_file, cache_status, 'UniformOutput', false);
end
