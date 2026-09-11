function info = plot_pseudo_truth(est, frame_times, opts)
%PLOT_PSEUDO_TRUTH  画【用目标编号(tid)连接起来的伪真值轨迹】，并打印诊断。
%   伪真值来源：优先使用 opts.fused_xyz/opts.fused_ids 的全量融合量测；
%   若未提供，则退回 est.assoc{k} 里的已关联量测。
%   把同一 tid 的量测按时间连成折线，即“按编号连接的伪真值轨迹”。
%
%   info = plot_pseudo_truth(est, frame_times, opts)
%   返回诊断信息 info（含找到的 tid 列表、各 tid 点数等）。
%
%   opts（全部可选）:
%     tids        [向量] 只画这些目标编号；留空=全部出现过的tid        默认 []
%     track_ids   [向量] 用这些航迹选出关联tid，并画这些tid的完整轨迹     默认 []
%     tid_select_mode  'all_touched'=画碰过的全部tid；'dominant'=只画主导tid 默认 'all_touched'
%     cfg         配置结构；提供后可按 cfg.truth_id_split_* 绘制拆分后的伪真值 默认 []
%     truth_use_split  true=使用拆分实例编号；false=使用原始tid；留空=跟随cfg 默认 []
%     fused_xyz   {K×1} 全量融合量测位置，优先用它画完整伪真值轨迹       默认 {}
%     fused_ids   {K×1} 与 fused_xyz 列对应的目标编号                  默认 {}
%     show_meas   叠加全部关联量测点(浅灰)                              默认 true
%     show_filter 叠加一次滤波轨迹(细虚线,灰)作对照                     默认 false
%     line_width  伪真值线宽                                          默认 2.0
%     meas_color  量测颜色                                            默认 [0.7 0.7 0.7]
%     no_figure   true=只返回伪真值轨迹数据，不创建3D图                 默认 false
%     events      联合二维/三维事件；提供后同时返回完整角度和ENU真值序列 默认 []
%     truth_all   true=忽略航迹选择，返回数据中的全部真值目标           默认 false

if nargin < 3 || isempty(opts), opts = struct(); end
gp = @(f, d) ppt_get(opts, f, d);
K = numel(frame_times);
joint_events = gp('events', []);
if isstruct(joint_events) && ~isempty(joint_events)
    info = collect_joint_truth(est, joint_events, opts);
    return;
end

%% —— 诊断：assoc / tid 是否可用 ——
info = struct('has_assoc', false, 'frames_with_assoc', 0, ...
              'n_points', 0, 'n_finite_tid', 0, ...
              'n_assoc_points', 0, 'n_assoc_finite_tid', 0, ...
              'truth_source', '', 'frames_with_truth_source', 0, ...
              'truth_use_split', false, 'truth_split_info', [], ...
              'track_tid_summary', [], ...
              'tids', [], ...
              'pts_per_tid', [], 'TRk', repmat(struct('tid', NaN, ...
              't', [], 'p', zeros(3, 0)), 1, 0));
track_ids = gp('track_ids', []);
if ~isempty(track_ids)
    track_ids = unique(track_ids(:).');
end
fused_xyz = gp('fused_xyz', {});
fused_ids = gp('fused_ids', {});
cfg = gp('cfg', []);
has_assoc = isfield(est, 'assoc') && ~isempty(est.assoc);
has_full_truth = iscell(fused_xyz) && iscell(fused_ids) && ...
                 ~isempty(fused_xyz) && ~isempty(fused_ids);
truth_use_split = gp('truth_use_split', []);
if isempty(truth_use_split)
    truth_use_split = isstruct(cfg) && isfield(cfg, 'truth_id_split_enabled') && ...
        ~isempty(cfg.truth_id_split_enabled) && cfg.truth_id_split_enabled ~= 0;
end
truth_use_split = truth_use_split ~= 0 && has_full_truth;
truth_ids_full = fused_ids;
truth_split_info = [];
if truth_use_split
    [truth_ids_full, truth_split_info] = build_split_truth_labels(frame_times, fused_xyz, fused_ids, cfg);
end
if ~has_assoc && ~has_full_truth
    fprintf(['[伪真值诊断] 没有 est.assoc，也没有 opts.fused_xyz/opts.fused_ids，' ...
             '无法构建按编号的伪真值。\n']);
    return;
end
info.has_assoc = has_assoc;
info.truth_use_split = truth_use_split;
info.truth_split_info = truth_split_info;

% est.assoc 只用于两件事：
% 1) 统计已关联量测；
% 2) 当传入 track_ids 时，确定这些滤波航迹主关联到哪些 tid。
AssocT = zeros(1, 0); AssocTid = zeros(1, 0); AssocP = zeros(3, 0);
TrackTid = zeros(1, 0);
track_id_map = [];
TrackTidVotes = cell(1, numel(track_ids));
if ~isempty(track_ids)
    track_id_map = containers.Map('KeyType', 'double', 'ValueType', 'double');
    for s = 1:numel(track_ids)
        track_id_map(track_ids(s)) = s;
        TrackTidVotes{s} = zeros(1, 0);
    end
end
assoc_pts = 0; assoc_fin = 0; assoc_frames = 0;
if has_assoc
    K_assoc = min(K, numel(est.assoc));
    for k = 1:K_assoc
        As = est.assoc{k};
        if isempty(As) || ~isfield(As, 'xyz') || isempty(As.xyz), continue; end
        assoc_frames = assoc_frames + 1;
        ncol = size(As.xyz, 2);
        has_tid = isfield(As, 'tid') && ~isempty(As.tid);
        has_id  = isfield(As, 'id')  && ~isempty(As.id);
        for c = 1:ncol
            assoc_pts = assoc_pts + 1;
            if ~has_tid || c > numel(As.tid), continue; end
            raw_tc = As.tid(c);
            if ~isfinite(raw_tc), continue; end
            if truth_use_split
                tc = map_assoc_to_plot_id(k, As.xyz(1:3, c), raw_tc, fused_xyz, fused_ids, truth_ids_full);
            else
                tc = raw_tc;
            end
            if ~isfinite(tc), continue; end
            assoc_fin = assoc_fin + 1;
            AssocT(end+1) = frame_times(k);        %#ok<AGROW>
            AssocTid(end+1) = tc;                  %#ok<AGROW>
            AssocP(:, end+1) = As.xyz(1:3, c);     %#ok<AGROW>
            if ~isempty(track_ids) && has_id && c <= numel(As.id)
                if ismember(As.id(c), track_ids(:)')
                    TrackTid(end+1) = tc;          %#ok<AGROW>
                    if ~isempty(track_id_map) && isKey(track_id_map, As.id(c))
                        s = track_id_map(As.id(c));
                        TrackTidVotes{s}(end+1) = tc;
                    end
                end
            end
        end
    end
else
    fprintf('[伪真值诊断] est.assoc 不可用；只能在显式指定 tids 或画全部 tid 时使用全量伪真值。\n');
end
info.frames_with_assoc = assoc_frames;
info.n_assoc_points = assoc_pts;
info.n_assoc_finite_tid = assoc_fin;

% 真正用于画线的数据源：优先全量融合量测。这样选中某条滤波航迹后，
% 只要它投票到了 tid，就画这个 tid 在 fused_ids 中出现过的全部量测点。
AllT = zeros(1, 0); AllTid = zeros(1, 0); AllP = zeros(3, 0);
n_pts = 0; n_fin = 0;
frames_with_source = 0;
truth_source = '已关联量测(est.assoc)';
if has_full_truth
    K_full = min([K, numel(fused_xyz), numel(fused_ids)]);
    for k = 1:K_full
        P = fused_xyz{k};
        ids = truth_ids_full{k};
        if isempty(P) || isempty(ids) || size(P, 1) < 3, continue; end
        ids = ids(:).';
        ncol = min(numel(ids), size(P, 2));
        if ncol <= 0, continue; end
        frames_with_source = frames_with_source + 1;
        for c = 1:ncol
            n_pts = n_pts + 1;
            tc = ids(c);
            if ~isfinite(tc), continue; end
            n_fin = n_fin + 1;
            AllT(end+1) = frame_times(k);      %#ok<AGROW>
            AllTid(end+1) = tc;                %#ok<AGROW>
            AllP(:, end+1) = P(1:3, c);        %#ok<AGROW>
        end
    end
    if truth_use_split
        truth_source = '全量融合量测拆分实例(fused_xyz/fused_ids + truth split)';
        fprintf('[伪真值诊断] 绘图使用拆分后伪真值实例：原始编号=%d，拆分实例=%d，被拆分原始编号=%d\n', ...
            truth_split_info.raw_id_count, truth_split_info.instance_count, truth_split_info.n_split_raw_ids);
    else
        truth_source = '全量融合量测(fused_xyz/fused_ids)';
    end
    if n_fin == 0 && assoc_fin > 0
        fprintf('[伪真值诊断] fused_ids 中没有有效 tid，退回使用 est.assoc 已关联量测。\n');
        AllT = AssocT; AllTid = AssocTid; AllP = AssocP;
        n_pts = assoc_pts; n_fin = assoc_fin; frames_with_source = assoc_frames;
        truth_source = '已关联量测(est.assoc)';
    end
else
    AllT = AssocT; AllTid = AssocTid; AllP = AssocP;
    n_pts = assoc_pts; n_fin = assoc_fin; frames_with_source = assoc_frames;
end
info.n_points = n_pts;
info.n_finite_tid = n_fin;
info.frames_with_truth_source = frames_with_source;
info.truth_source = truth_source;

if has_assoc
    fprintf('[伪真值诊断] 航迹-tid映射来源=est.assoc：有关联帧数=%d，关联量测点=%d，其中带有效编号(tid)的=%d\n', ...
            assoc_frames, assoc_pts, assoc_fin);
end
fprintf('[伪真值诊断] 伪真值画线来源=%s：帧数=%d，量测点=%d，其中带有效编号(tid)的=%d\n', ...
        truth_source, frames_with_source, n_pts, n_fin);
if ~isempty(track_ids) && isempty(TrackTid)
    fprintf('[伪真值诊断] 选中航迹在 est.assoc 中没有投到任何 tid；无法自动确定要画哪条伪真值。\n');
end
if n_fin == 0
    fprintf(['[伪真值诊断] 没有任何带“目标编号(tid)”的量测——' ...
             '说明数据里量测没有目标编号(全是NaN)，因此无法画“按编号连接的伪真值”。\n']);
    return;
end

%% —— 选定要画的 tid ——
tids_opt = gp('tids', []);
truth_all = gp('truth_all', false) ~= 0;
tid_select_mode = lower(gp('tid_select_mode', 'all_touched'));
if ~ismember(tid_select_mode, {'all_touched', 'dominant'})
    fprintf('[伪真值诊断] 未知 tid_select_mode=%s，按 all_touched 处理。\n', tid_select_mode);
    tid_select_mode = 'all_touched';
end
available_tids = unique(AllTid(isfinite(AllTid)));
if truth_all
    use_tids = available_tids;
elseif isempty(tids_opt)
    if ~isempty(track_ids)
        use_tids = zeros(1, 0);
        summary = repmat(struct('track_id', NaN, 'main_tid', NaN, ...
            'main_count', 0, 'total_count', 0, 'touched_tids', [], 'counts', []), ...
            1, numel(track_ids));
        for s = 1:numel(track_ids)
            votes = TrackTidVotes{s};
            votes = votes(isfinite(votes));
            summary(s).track_id = track_ids(s);
            summary(s).total_count = numel(votes);
            if isempty(votes)
                continue;
            end
            u = unique(votes);
            cnt = zeros(size(u));
            for ii = 1:numel(u)
                cnt(ii) = sum(votes == u(ii));
            end
            [main_count, im] = max(cnt);
            main_tid = u(im);
            summary(s).main_tid = main_tid;
            summary(s).main_count = main_count;
            summary(s).touched_tids = u;
            summary(s).counts = cnt;
            if strcmp(tid_select_mode, 'dominant')
                add_tids = main_tid;
            else
                add_tids = u;
            end
            add_tids = intersect(add_tids, available_tids);
            if ~isempty(add_tids)
                use_tids = [use_tids, add_tids]; %#ok<AGROW>
            end
            if numel(u) > 1
                fprintf('[伪真值诊断] 航迹ID=%g 碰过tid=%s，对应次数=%s；主导tid=%g (%d/%d)；绘制模式=%s\n', ...
                    track_ids(s), mat2str(u), mat2str(cnt), ...
                    main_tid, main_count, numel(votes), tid_select_mode);
            else
                fprintf('[伪真值诊断] 航迹ID=%g 碰过tid=%g；主导tid=%g (%d/%d)；绘制模式=%s\n', ...
                    track_ids(s), main_tid, main_tid, main_count, numel(votes), tid_select_mode);
            end
        end
        use_tids = unique(use_tids);
        info.track_tid_summary = summary;
    else
        use_tids = available_tids;
    end
else
    use_tids = intersect(unique(tids_opt(:)'), available_tids);
end
if isempty(use_tids)
    if isempty(tids_opt) && ~isempty(track_ids)
        fprintf('[伪真值诊断] 选中航迹没有可画的关联 tid。可用目标编号为：%s\n', mat2str(available_tids));
    else
        fprintf('[伪真值诊断] 指定的 tids 在数据中不存在。可用编号为：%s\n', mat2str(available_tids));
    end
    return;
end
if truth_use_split
    id_label = '拆分实例编号';
else
    id_label = '目标编号';
end
fprintf('[伪真值诊断] 可用%s：%s；本次绘制：%s\n', id_label, ...
        mat2str(available_tids), mat2str(use_tids));

%% —— 逐 tid 连成折线（同一时刻多点取均值，按时间排序）——
nT = numel(use_tids);
TRk = repmat(struct('tid', NaN, 't', [], 'p', zeros(3, 0)), 1, nT);
pts_per_tid = zeros(1, nT);
for i = 1:nT
    sel = (AllTid == use_tids(i));
    tt = AllT(sel); pp = AllP(:, sel);
    [ut, ~, ic] = unique(tt);
    pm = zeros(3, numel(ut));
    for j = 1:numel(ut), pm(:, j) = mean(pp(:, ic == j), 2); end
    [ut, ord] = sort(ut); pm = pm(:, ord);
    TRk(i) = struct('tid', use_tids(i), 't', ut, 'p', pm);
    pts_per_tid(i) = numel(ut);
end
info.tids = use_tids; info.pts_per_tid = pts_per_tid; info.TRk = TRk;

%% —— 绘图 ——
if gp('no_figure', false) ~= 0
    return;
end

line_width = gp('line_width', 2.0);
show_meas  = gp('show_meas', true) ~= 0;
show_filt  = gp('show_filter', false) ~= 0;
meas_color = gp('meas_color', [0.70 0.70 0.70]);

cols = lines(max(nT, 1));
fig = figure('Name', '按目标编号连接的伪真值轨迹', 'Color', 'w', 'Position', [80, 80, 1000, 800]);
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

% 量测背景
if show_meas
    plot3(ax, AllP(1, :), AllP(2, :), AllP(3, :), '.', 'Color', meas_color, 'MarkerSize', 5);
end

% 一次滤波轨迹（可选，灰色细虚线对照）
if show_filt && isfield(est, 'tracks')
    plot_filter_overlay(ax, est, K);
end

% 伪真值折线
hL = gobjects(1, nT);
for i = 1:nT
    hL(i) = plot3(ax, TRk(i).p(1, :), TRk(i).p(2, :), TRk(i).p(3, :), '-', ...
                  'Color', cols(i, :), 'LineWidth', line_width);
    plot3(ax, TRk(i).p(1, 1), TRk(i).p(2, 1), TRk(i).p(3, 1), 'o', ...
          'MarkerEdgeColor', 'k', 'MarkerFaceColor', cols(i, :), 'MarkerSize', 5);
    text(ax, TRk(i).p(1, 1), TRk(i).p(2, 1), TRk(i).p(3, 1), ...
         sprintf(' %s', truth_label_short(use_tids(i), truth_split_info, truth_use_split)), ...
         'Color', cols(i, :), 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
end

xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Up (m)');
view(ax, 45, 30); axis(ax, 'equal');
labs = arrayfun(@(t) truth_label_long(t, truth_split_info, truth_use_split), ...
    use_tids, 'UniformOutput', false);
legend(ax, hL, labs, 'Location', 'bestoutside');
title(ax, sprintf('按目标编号连接的伪真值轨迹 (共%d条)', nT));
hold(ax, 'off');
end

%% ═══════════════════════════════════════════════════════════════════════════
function info = collect_joint_truth(est, events, opts)
cfg = ppt_get(opts, 'cfg', struct());
use_split = ppt_get(opts, 'truth_use_split', []);
if isempty(use_split)
    use_split = isstruct(cfg) && isfield(cfg, 'truth_id_split_enabled') && ...
        ~isempty(cfg.truth_id_split_enabled) && cfg.truth_id_split_enabled ~= 0;
end
cfg.truth_id_split_enabled = logical(use_split);
labels = build_joint_truth_labels(events, cfg);
all_keys = labels.summary.instance_labels;
[all_tracks, n_points, n_frames] = joint_truth_series(events, labels);

track_ids = unique(reshape(ppt_get(opts, 'track_ids', []), 1, []));
summaries = joint_track_truth_summary(est, events, labels, track_ids);
mode = lower(ppt_get(opts, 'tid_select_mode', 'all_touched'));
if ~ismember(mode, {'all_touched', 'dominant'})
    fprintf('[伪真值诊断] 未知 tid_select_mode=%s，按 all_touched 处理。\n', mode);
    mode = 'all_touched';
end
explicit = reshape(ppt_get(opts, 'tids', []), 1, []);
truth_all = ppt_get(opts, 'truth_all', false) ~= 0;
if truth_all || (isempty(track_ids) && isempty(explicit))
    selected_keys = all_keys;
elseif ~isempty(explicit)
    selected_keys = intersect(unique(explicit(isfinite(explicit))), all_keys);
else
    selected_keys = zeros(1, 0);
    for i = 1:numel(summaries)
        if strcmp(mode, 'dominant')
            add = summaries(i).main_tid;
        else
            add = summaries(i).touched_tids;
        end
        selected_keys = [selected_keys, add(isfinite(add))]; %#ok<AGROW>
    end
    selected_keys = intersect(unique(selected_keys), all_keys);
end

keep = ismember([all_tracks.tid], selected_keys);
tracks = all_tracks(keep);
info = struct('has_assoc', isfield(est, 'assoc') && ~isempty(est.assoc), ...
    'frames_with_assoc', 0, 'n_points', n_points, 'n_finite_tid', n_points, ...
    'n_assoc_points', sum([summaries.total_count]), ...
    'n_assoc_finite_tid', sum([summaries.total_count]), ...
    'truth_source', '联合二维/三维事件(joint events)', ...
    'frames_with_truth_source', n_frames, ...
    'truth_use_split', logical(use_split), 'truth_split_info', labels.summary, ...
    'track_tid_summary', summaries, 'tid_select_mode', mode, ...
    'tids', selected_keys, 'pts_per_tid', arrayfun(@joint_track_point_count, tracks), ...
    'TRk', tracks);
if info.has_assoc
    info.frames_with_assoc = nnz(cellfun(@(x) ~isempty(x), est.assoc));
end

fprintf('[伪真值诊断] 联合事件完整真值：可用=%s，本次绘制=%s，模式=%s。\n', ...
    mat2str(all_keys), mat2str(selected_keys), mode);
for i = 1:numel(summaries)
    if numel(summaries(i).touched_tids) > 1
        fprintf('[伪真值诊断] 航迹ID=%g 碰过tid=%s，对应次数=%s；主导tid=%g (%d/%d)。\n', ...
            summaries(i).track_id, mat2str(summaries(i).touched_tids), ...
            mat2str(summaries(i).counts), summaries(i).main_tid, ...
            summaries(i).main_count, summaries(i).total_count);
    end
end

if ppt_get(opts, 'no_figure', false) ~= 0, return; end
plot_joint_truth_figures(tracks, labels.summary, logical(use_split), ...
    ppt_get(opts, 'line_width', 2.0));
end

function [tracks, n_points, n_frames] = joint_truth_series(events, labels)
keys = labels.summary.instance_labels;
template = struct('tid', NaN, 'label', '', 't', zeros(1, 0), ...
    'p', zeros(3, 0), 'angle_t', zeros(1, 0), 'angle', zeros(2, 0));
tracks = repmat(template, 1, numel(keys));
for i = 1:numel(keys)
    tracks(i).tid = keys(i);
    tracks(i).label = joint_truth_label(labels.summary, keys(i));
end
n_points = 0; n_frames = 0;
for k = 1:numel(events)
    e = events(k); used = false;
    na = e.active.n_meas; np = e.passive.n_meas;
    active_keys = joint_sized_row(labels.active{k}, na, NaN);
    passive_keys = joint_sized_row(labels.passive{k}, np, NaN);
    active_t = joint_measurement_times(e.active.t_sec, na, e.t_sec);
    passive_t = joint_measurement_times(e.passive.t_sec, np, e.t_sec);
    for j = 1:na
        slot = find(keys == active_keys(j), 1);
        if isempty(slot), continue; end
        if size(e.active.rae, 1) >= 3 && j <= size(e.active.rae, 2) && ...
                all(isfinite(e.active.rae(2:3, j)))
            tracks(slot).angle_t(end + 1) = active_t(j);
            tracks(slot).angle(:, end + 1) = e.active.rae(2:3, j);
            n_points = n_points + 1; used = true;
        end
        has_range = j <= numel(e.active.has_range) && e.active.has_range(j);
        if has_range && size(e.active.xyz, 1) >= 3 && j <= size(e.active.xyz, 2) && ...
                all(isfinite(e.active.xyz(1:3, j)))
            tracks(slot).t(end + 1) = active_t(j);
            tracks(slot).p(:, end + 1) = e.active.xyz(1:3, j);
        end
    end
    for j = 1:np
        slot = find(keys == passive_keys(j), 1);
        if isempty(slot) || size(e.passive.ang, 1) < 2 || ...
                j > size(e.passive.ang, 2) || any(~isfinite(e.passive.ang(1:2, j)))
            continue;
        end
        tracks(slot).angle_t(end + 1) = passive_t(j);
        tracks(slot).angle(:, end + 1) = e.passive.ang(1:2, j);
        n_points = n_points + 1; used = true;
    end
    n_frames = n_frames + used;
end
for i = 1:numel(tracks)
    [tracks(i).angle_t, tracks(i).angle] = joint_merge_angle( ...
        tracks(i).angle_t, tracks(i).angle);
    [tracks(i).t, tracks(i).p] = joint_merge_linear(tracks(i).t, tracks(i).p);
end
end

function summary = joint_track_truth_summary(est, events, labels, track_ids)
summary = repmat(struct('track_id', NaN, 'main_tid', NaN, ...
    'main_count', 0, 'total_count', 0, 'touched_tids', [], 'counts', []), ...
    1, numel(track_ids));
votes = cell(1, numel(track_ids));
for i = 1:numel(track_ids)
    summary(i).track_id = track_ids(i); votes{i} = zeros(1, 0);
end
if isempty(track_ids), return; end
for k = 1:numel(events)
    a = joint_event_assoc(est, k);
    for q = 1:numel(a.id)
        i = find(track_ids == a.id(q), 1);
        if isempty(i), continue; end
        key = joint_assoc_truth_key(a, q, labels, k);
        if isfinite(key), votes{i}(end + 1) = key; end
    end
end
for i = 1:numel(track_ids)
    if isempty(votes{i})
        votes{i} = joint_output_truth_votes(est, events, labels, track_ids(i));
    end
    u = unique(votes{i}(isfinite(votes{i}))); cnt = zeros(size(u));
    for j = 1:numel(u), cnt(j) = nnz(votes{i} == u(j)); end
    summary(i).total_count = sum(cnt); summary(i).touched_tids = u;
    summary(i).counts = cnt;
    if ~isempty(cnt)
        [summary(i).main_count, j] = max(cnt);
        summary(i).main_tid = u(j);
    end
end
end

function a = joint_event_assoc(est, k)
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, ...
    'meas_index', zeros(1, 0), 'tid', zeros(1, 0));
if isfield(est, 'assoc') && k <= numel(est.assoc) && ~isempty(est.assoc{k})
    a = est.assoc{k};
end
end

function key = joint_assoc_truth_key(a, q, labels, k)
key = NaN;
if isfield(a, 'type') && q <= numel(a.type) && ...
        isfield(a, 'meas_index') && q <= numel(a.meas_index)
    type = a.type{q}; mi = a.meas_index(q);
    if isstring(type) && isscalar(type), type = char(type); end
    if ischar(type) && isfinite(mi) && mi >= 1 && mi == round(mi)
        if strncmp(type, 'active', 6), x = labels.active{k};
        elseif strncmp(type, 'passive', 7), x = labels.passive{k};
        else, x = zeros(1, 0);
        end
        if mi <= numel(x), key = x(mi); end
    end
end
if isfinite(key) || ~isfield(a, 'tid') || q > numel(a.tid) || ~isfinite(a.tid(q))
    return;
end
raw = a.tid(q); summary = labels.summary;
candidates = summary.instance_labels(summary.key_raw_id == raw);
if numel(candidates) == 1, key = candidates; end
end

function votes = joint_output_truth_votes(est, events, labels, track_id)
votes = zeros(1, 0);
if ~isfield(est, 'output'), return; end
for k = 1:min(numel(events), numel(est.output))
    out = est.output{k};
    for q = find([out.id] == track_id)
        if ~isfield(out(q), 'truth_id') || ~isfinite(out(q).truth_id), continue; end
        raw = out(q).truth_id; candidates = labels.summary.instance_labels( ...
            labels.summary.key_raw_id == raw);
        if numel(candidates) == 1, votes(end + 1) = candidates; end %#ok<AGROW>
    end
end
end

function plot_joint_truth_figures(tracks, split_info, use_split, line_width)
has_position = arrayfun(@(x) ~isempty(x.t), tracks);
if any(has_position)
    fig = figure('Name', '按目标编号连接的伪真值轨迹', 'Color', 'w', ...
        'Position', [80, 80, 1000, 800]);
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    colors = lines(nnz(has_position)); q = 0;
    for i = find(has_position)
        q = q + 1;
        plot3(ax, tracks(i).p(1, :), tracks(i).p(2, :), tracks(i).p(3, :), '-', ...
            'Color', colors(q, :), 'LineWidth', line_width, ...
            'DisplayName', truth_label_long(tracks(i).tid, split_info, use_split));
    end
    xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Up (m)');
    view(ax, 45, 30); axis(ax, 'equal'); legend(ax, 'Location', 'bestoutside');
    title(ax, sprintf('按目标编号连接的完整伪真值轨迹 (共%d条)', nnz(has_position)));
end
has_angle = arrayfun(@(x) ~isempty(x.angle_t), tracks);
if any(has_angle)
    figure('Name', '按目标编号连接的二维角度伪真值', 'Color', 'w', ...
        'Position', [110, 110, 900, 700]);
    hold on; grid on; box on; colors = lines(nnz(has_angle)); q = 0;
    for i = find(has_angle)
        q = q + 1;
        plot(tracks(i).angle(1, :), tracks(i).angle(2, :), '-', ...
            'Color', colors(q, :), 'LineWidth', line_width, ...
            'DisplayName', truth_label_long(tracks(i).tid, split_info, use_split));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    title(sprintf('按目标编号连接的完整二维角度伪真值 (共%d条)', nnz(has_angle)));
    legend('Location', 'bestoutside'); hold off;
end
end

function n = joint_track_point_count(track)
n = max(numel(track.t), numel(track.angle_t));
end

function label = joint_truth_label(summary, key)
label = sprintf('%g', key); i = find(summary.instance_labels == key, 1);
if ~isempty(i) && i <= numel(summary.instance_label_text)
    label = summary.instance_label_text{i};
end
end

function [t, z] = joint_merge_angle(t, z)
if isempty(t), t = zeros(1, 0); z = zeros(2, 0); return; end
[t, order] = sort(t(:).'); z = z(:, order); [u, ~, group] = unique(t, 'stable');
merged = nan(2, numel(u));
for i = 1:numel(u)
    q = group == i;
    merged(:, i) = [atan2d(mean(sind(z(1, q))), mean(cosd(z(1, q)))); mean(z(2, q))];
end
t = u; z = merged;
end

function [t, z] = joint_merge_linear(t, z)
if isempty(t), t = zeros(1, 0); z = zeros(size(z, 1), 0); return; end
[t, order] = sort(t(:).'); z = z(:, order); [u, ~, group] = unique(t, 'stable');
merged = nan(size(z, 1), numel(u));
for i = 1:numel(u), merged(:, i) = mean(z(:, group == i), 2); end
t = u; z = merged;
end

function x = joint_sized_row(x0, n, fill)
x = fill * ones(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0)); x(1:m) = reshape(x0(1:m), 1, []);
end

function t = joint_measurement_times(t0, n, fallback)
t = fallback * ones(1, n);
if n == 0 || isempty(t0), return; end
if isscalar(t0), t(:) = t0; else, m = min(n, numel(t0)); t(1:m) = t0(1:m); end
end

%% ═══════════════════════════════════════════════════════════════════════════
function plot_filter_overlay(ax, est, K)
pos_idx = [1, 4, 7];
seen = containers.Map('KeyType', 'double', 'ValueType', 'any');
for k = 1:K
    Tr = est.tracks{k};
    if isempty(Tr) || ~isfield(Tr, 'L') || isempty(Tr.L), continue; end
    ids = Tr.L(2, :);
    for c = 1:numel(ids)
        key = ids(c);
        if ~isKey(seen, key), seen(key) = zeros(3, 0); end
        v = seen(key); v(:, end+1) = Tr.m(pos_idx, c); seen(key) = v; %#ok<AGROW>
    end
end
ks = seen.keys;
for i = 1:numel(ks)
    p = seen(ks{i});
    plot3(ax, p(1, :), p(2, :), p(3, :), '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.8);
    annotate_track_start(ax, p, ks{i}, [0.40 0.40 0.40]);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function plot_id = map_assoc_to_plot_id(k, pos, raw_id, fused_xyz, fused_ids, truth_ids_full)
plot_id = nan;
if k > numel(fused_xyz) || k > numel(fused_ids) || k > numel(truth_ids_full)
    return;
end
Z = fused_xyz{k};
raw = fused_ids{k};
sid = truth_ids_full{k};
if isempty(Z) || isempty(raw) || isempty(sid) || size(Z, 1) < 3
    return;
end
raw = raw(:).';
sid = sid(:).';
n = min([size(Z, 2), numel(raw), numel(sid)]);
if n <= 0
    return;
end
idx = find(raw(1:n) == raw_id & isfinite(sid(1:n)));
if isempty(idx)
    return;
end
d = sqrt(sum((Z(1:3, idx) - pos(:)).^2, 1));
[~, im] = min(d);
plot_id = sid(idx(im));
end

function txt = truth_label_short(key, split_info, use_split)
if use_split && isstruct(split_info) && isfield(split_info, 'key_raw_id') && ...
        key >= 1 && key <= numel(split_info.key_raw_id) && isfinite(split_info.key_raw_id(key))
    txt = sprintf('tid=%g-%g', split_info.key_raw_id(key), split_info.key_instance(key));
else
    txt = sprintf('tid=%g', key);
end
end

function txt = truth_label_long(key, split_info, use_split)
if use_split && isstruct(split_info) && isfield(split_info, 'key_raw_id') && ...
        key >= 1 && key <= numel(split_info.key_raw_id) && isfinite(split_info.key_raw_id(key))
    txt = sprintf('伪真值 key=%g (原tid=%g, 实例=%g)', ...
        key, split_info.key_raw_id(key), split_info.key_instance(key));
else
    txt = sprintf('伪真值 tid=%g', key);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = ppt_get(opts, f, d)
if isfield(opts, f) && ~isempty(opts.(f)), v = opts.(f); else, v = d; end
end
