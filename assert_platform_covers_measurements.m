function assert_platform_covers_measurements(platform, active_list, passive_list)
%ASSERT_PLATFORM_COVERS_MEASUREMENTS Verify every selected measurement time.
first_t = inf;
last_t = -inf;
lists = [active_list(:); passive_list(:)];
for i = 1:numel(lists)
    if ~isfield(lists{i}, 't_sec'), continue; end
    times = lists{i}.t_sec(:);
    times = times(isfinite(times));
    if isempty(times), continue; end
    first_t = min(first_t, min(times));
    last_t = max(last_t, max(times));
end
if ~isfinite(first_t), return; end

query = [first_t, last_t];
platform.interp_lat(query);
platform.interp_lon(query);
platform.interp_alt(query);
t0 = min(platform.t_sec);
t1 = max(platform.t_sec);
before_s = max(t0 - first_t, 0);
after_s = max(last_t - t1, 0);
if before_s > 1e-9 || after_s > 1e-9
    fprintf(['[平台覆盖] 使用端点线性外推：起点=%.6fs，终点=%.6fs，' ...
        '上限=%.6fs。\n'], before_s, after_s, platform.max_extrapolation_s);
end
end
