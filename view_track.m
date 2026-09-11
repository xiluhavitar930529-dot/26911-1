clear play_track_filter plot_track_filter_smooth plot_pseudo_truth smooth_filter_tracks get_smoothed_track
rehash

TRACK_IDS    = [2];          % 要看的航迹ID：单条填 142；多条填 [3 7 142]；填 [] = 全部确认航迹
DO_PLAYBACK  = false;         % 打开 play_track_filter 回放窗口
DO_COMPARE   = true;         % 出“一次滤波 vs 二次平滑 + 量测”对比图
DO_TRUTH     = true;         % 出“按目标编号连接的伪真值轨迹”单独图(最可靠)
DO_ENU_TIME  = true;         % 出“滤波估值 vs 伪真值”东北天三轴时间序列图
DO_TRACK_REPORT = true;      % 控制台打印单轨评价，并为每个TRACK_IDS输出TXT明细
TRACK_REPORT_DIR = fullfile(fileparts(mfilename('fullpath')), 'track_reports');

% —— 回放窗口参数 ——
SHOW_TRUTH   = true;         % 回放里叠加伪真值线（新版 play_track_filter 已修好，可放心开）
MODE         = 'slider';     % 'slider'交互 或 'play'自动播放
SHOW_MEAS    = true;
SHOW_COV     = true;
SHOW_GATE    = true;
TRAIL_MODE   = 'cumulative';
TRAIL_WIN_S  = 10;

% —— 伪真值显示(回放叠加用)——
TRUTH_FULL   = true;         % true=整条静态显示(推荐,随时可见) / false=随时间揭示
TRUTH_TINT   = 0.65;         % 颜色深浅(0~1,越大越深越显眼)，原来0.3太浅
TRUTH_WIDTH  = 2.0;          % 伪真值线宽
TRUTH_SELECT_MODE = 'all_touched'; % 'dominant'=主导真值；'all_touched'=所有关联过的真值
TRUTH_ALL    = false;     % true=忽略上一项并画数据中的全部真值目标
TRUTH_USE_SPLIT = false;        % []=跟随cfg.truth_id_split_enabled；true=画拆分实例；false=画原始tid

% —— 二次平滑参数(对比图用)——
SMOOTH_METHOD = 'fixedlag';
SMOOTH_LAG    = 8;
FILTER_WIDTH  = 1.2;
SMOOTH_WIDTH  = 3.0;

RESULT_MAT   = '';           % 若 est/frame_times 不在工作区，从此 .mat 读取；留空=不读取
% ========================================================================

%% —— 准备 est / frame_times ——
have = exist('est', 'var') && exist('frame_times', 'var');
if ~have && ~isempty(RESULT_MAT)
    if exist(RESULT_MAT, 'file')
        Sload = load(RESULT_MAT);
        if isfield(Sload, 'cfg'), cfg = Sload.cfg; end
        if isfield(Sload, 'est'), est = Sload.est; end
        if isfield(Sload, 'frame_times'), frame_times = Sload.frame_times; end
        if isfield(Sload, 'fused_xyz'), fused_xyz = Sload.fused_xyz; end
        if isfield(Sload, 'fused_ids'), fused_ids = Sload.fused_ids; end
        if isfield(Sload, 'joint_events'), joint_events = Sload.joint_events; end
        have = exist('est', 'var') && exist('frame_times', 'var');
    else
        error('找不到 RESULT_MAT 指定的文件：%s', RESULT_MAT);
    end
end
if ~have
    error(['工作区没有 est / frame_times。请先运行 run_fusion_main，' ...
           '或把 RESULT_MAT 填成保存结果的 .mat 路径后再运行本脚本。']);
end
if isfield(est, 'framework') && strncmp(est.framework, 'joint_2d3d', 10)
    if ~exist('joint_events', 'var') || isempty(joint_events)
        joint_events = est.event_meta;
    end
    jo = struct('track_ids', TRACK_IDS, 'show_reference', DO_TRUTH, ...
        'show_angle', true, 'show_enu', DO_ENU_TIME, ...
        'truth_select_mode', TRUTH_SELECT_MODE, 'truth_all', TRUTH_ALL);
    if exist('cfg', 'var') && isstruct(cfg)
        jo.cfg = cfg;
    end
    if ~isempty(TRUTH_USE_SPLIT)
        jo.truth_use_split = logical(TRUTH_USE_SPLIT);
    end
    if DO_TRACK_REPORT && ~isempty(TRACK_IDS)
        report_opts = struct('output_dir', TRACK_REPORT_DIR, 'print_console', true);
        report_cfg = struct();
        if exist('cfg', 'var') && isstruct(cfg), report_cfg = cfg; end
        if exist('metrics_info', 'var') && isstruct(metrics_info)
            report_opts.metrics = metrics_info;
        end
        try
            export_track_diagnostics(est, joint_events, ...
                report_cfg, TRACK_IDS, report_opts);
        catch ME
            fprintf(2, '[单轨报告失败] %s\n', ME.message);
        end
    elseif DO_TRACK_REPORT
        fprintf('[单轨报告] TRACK_IDS=[] 表示总览，本次不批量导出全部航迹。\n');
    end
    view_joint_tracks(est, joint_events, jo);
    return;
end
have_full_truth = exist('fused_xyz', 'var') && exist('fused_ids', 'var') && ...
                  iscell(fused_xyz) && iscell(fused_ids) && ...
                  ~isempty(fused_xyz) && ~isempty(fused_ids);
have_cfg = exist('cfg', 'var') && isstruct(cfg);
if have_full_truth
    fprintf('[伪真值] 检测到 fused_xyz/fused_ids：选中航迹投票到 tid 后，将绘制该 tid 的全量伪真值轨迹。\n');
else
    fprintf('[伪真值] 未检测到 fused_xyz/fused_ids：伪真值只能退回 est.assoc 已关联量测，可能不完整。\n');
end
if have_cfg && isfield(cfg, 'truth_id_split_enabled') && cfg.truth_id_split_enabled
    fprintf('[伪真值] 检测到 cfg.truth_id_split_enabled=true：伪真值图默认使用拆分后的实例编号。\n');
end

%% —— 自检：play_track_filter.m 是不是修复版、有没有多份 ——
allp = which('play_track_filter', '-all');
if ischar(allp), allp = {allp}; end
fprintf('\n[自检] MATLAB 找到的 play_track_filter.m（按加载优先级，只用第1个）：\n');
for ii = 1:numel(allp), fprintf('   %d) %s\n', ii, allp{ii}); end
if numel(allp) > 1
    fprintf(2, ['[自检] ★发现多份 play_track_filter.m！MATLAB 只用第1个，' ...
                '请把其余几份删除或改名。\n']);
end
if ~isempty(allp)
    try
        txt = fileread(allp{1});
        L = regexp(txt, '\r?\n', 'split');
        % 找到伪真值关键行：show_tids(end+1)=mode(sel_tid_votes{s})（旧）或安全版
        ko = find(~cellfun('isempty', regexp(L, 'mode\(sel_tid_votes', 'once')), 1);
        is_old = ~isempty(find(~cellfun('isempty', ...
                 regexp(L, 'for\s+s\s*=\s*1:n_sel.*sel_tid_votes|sel_tid_votes\{s\}', 'once')), 1)) ...
                 && ~isempty(find(~cellfun('isempty', ...
                 regexp(L, 'for\s+s\s*=\s*1:n_sel', 'once')), 1));
        has_safe = ~isempty(find(~cellfun('isempty', ...
                 regexp(L, "cellfun\('isempty', sel_tid_votes\)", 'once')), 1));
        if ~isempty(ko)
            lo = max(1, ko-2); hi = min(numel(L), ko+1);
            fprintf('[自检] 你的文件中“伪真值取tid”关键代码（第%d~%d行）：\n', lo, hi);
            for q = lo:hi, fprintf('   %d: %s\n', q, strtrim(L{q})); end
        end
        if has_safe
            fprintf('[自检] 该段为【最新安全版】(用 cellfun 遍历非空胞元)，不会越界。\n');
        else
            fprintf(2, ['[自检] ★你的 play_track_filter.m 这一段是【旧版】，回放会在此越界。\n' ...
                        '       请用最新文件【整文件覆盖】这个路径：\n          %s\n'], allp{1});
        end
    catch
    end
end

% 检查常用内置函数是否被项目里的同名文件顶替（这会导致 mode(...) 等报莫名越界错）
for fn = {'mode','unique','find','mean','sort','histc'}
    w = which(fn{1});
    if isempty(w), continue; end
    lw = lower(w);
    mr = lower(matlabroot);
    is_builtin = ~isempty(strfind(lw, 'built-in')); %#ok<STREMP>
    is_matlab_toolbox = strncmp(lw, mr, numel(mr));
    if ~is_builtin && ~is_matlab_toolbox
        fprintf(2, ['[自检] ★警告：内置函数 %s 被这个文件顶替了：\n          %s\n' ...
                    '       这很可能就是回放报“索引超出(6)”的真正原因！请把该文件改名或删除。\n'], fn{1}, w);
    end
end

%% —— 列出可用航迹ID，解析你要看的ID ——
avail_ids = [];
if isfield(est, 'L')
    for k = 1:numel(est.L)
        if ~isempty(est.L{k}), avail_ids = [avail_ids; est.L{k}(:, 2)]; end %#ok<AGROW>
    end
end
avail_ids = unique(avail_ids(:)');
if ~isempty(avail_ids)
    fprintf('可用确认航迹ID：'); fprintf(' %g', avail_ids); fprintf('\n');
end
if isempty(TRACK_IDS)
    sel_ids = avail_ids;
else
    if ~isempty(avail_ids)
        sel_ids = intersect(TRACK_IDS(:)', avail_ids);
        miss    = setdiff(TRACK_IDS(:)', avail_ids);
        if ~isempty(miss), fprintf('提示：以下指定ID不存在，已忽略：%s\n', mat2str(miss)); end
        if isempty(sel_ids)
            fprintf('指定的ID都不存在，改为显示全部确认航迹。\n'); sel_ids = avail_ids;
        end
    else
        sel_ids = TRACK_IDS(:)';
    end
end
fprintf('本次显示航迹ID：%s\n', mat2str(sel_ids));

%% —— 0) 伪真值单独图（最可靠，自带诊断打印）——
if DO_TRUTH
    to = struct();
    to.show_meas = SHOW_MEAS;
    to.tid_select_mode = TRUTH_SELECT_MODE;
    to.truth_all = TRUTH_ALL;
    if have_cfg
        to.cfg = cfg;
    end
    if ~isempty(TRUTH_USE_SPLIT), to.truth_use_split = TRUTH_USE_SPLIT; end
    if have_full_truth
        to.fused_xyz = fused_xyz;
        to.fused_ids = fused_ids;
    end
    to.track_ids = sel_ids;
    info = plot_pseudo_truth(est, frame_times, to); %#ok<NASGU>
end

%% —— 1) 回放窗口 ——
if DO_PLAYBACK
    pb = struct();
    pb.track_ids      = sel_ids;
    pb.show_truth     = SHOW_TRUTH;
    pb.truth_full     = TRUTH_FULL;
    pb.truth_tint     = TRUTH_TINT;
    pb.truth_line_width = TRUTH_WIDTH;
    if TRUTH_ALL, pb.truth_ids = []; end   % []=自动(取不到则兜底画全部)
    pb.mode           = MODE;
    pb.show_meas      = SHOW_MEAS;
    pb.show_cov       = SHOW_COV;
    pb.show_gate      = SHOW_GATE;
    pb.trail_mode     = TRAIL_MODE;
    pb.trail_window_s = TRAIL_WIN_S;
    if have_cfg
        pb.cfg = cfg;
    end
    if ~isempty(TRUTH_USE_SPLIT), pb.truth_use_split = TRUTH_USE_SPLIT; end
    if have_full_truth
        pb.fused_xyz = fused_xyz;
        pb.fused_ids = fused_ids;
    end
    try
        play_track_filter(est, frame_times, pb);
    catch ME
        fprintf(2, '\n[回放失败] %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf(2, '   实际出错位置（文件 / 行号）：\n');
            for q = 1:numel(ME.stack)
                fprintf(2, '      %s  第 %d 行\n', ME.stack(q).name, ME.stack(q).line);
            end
        end
        fprintf(2, ['   ★请把上面这几行“实际出错位置”发给开发者。\n' ...
                    '   若行号对应代码与最新文件对不上，说明 MATLAB 在跑缓存旧版本：\n' ...
                    '   请【完全关闭并重启 MATLAB】，重新 run_fusion_main 后再运行 view_track。\n' ...
                    '   （伪真值图与对比图不受影响，已正常生成。）\n']);
    end
end

%% —— 2) 一次滤波 vs 二次平滑 + 量测 对比图 ——
if DO_COMPARE
    co = struct();
    co.track_ids     = sel_ids;
    co.smooth_method = SMOOTH_METHOD;
    co.smooth_lag    = SMOOTH_LAG;
    co.filter_width  = FILTER_WIDTH;
    co.smooth_width  = SMOOTH_WIDTH;
    co.show_meas     = SHOW_MEAS;
    smt = plot_track_filter_smooth(est, frame_times, co); %#ok<NASGU>
end

%% —— 3) 滤波估值 vs 伪真值  ENU 时间序列图 ——
if DO_ENU_TIME
    n_sel = numel(sel_ids);
    if n_sel == 0
        fprintf('[ENU时间序列] 没有选中航迹ID，跳过。\n');
    else
        ax_lab = {'East (m)', 'North (m)', 'Up (m)'};
        fields = {'E', 'N', 'U'};

        idmap = containers.Map('KeyType', 'double', 'ValueType', 'double');
        for s = 1:n_sel
            idmap(sel_ids(s)) = s;
        end

        % -- 提取滤波估值：按选中航迹ID从 est.tracks 聚合 ENU 序列 --
        FILT = repmat(struct('t', [], 'E', [], 'N', [], 'U', []), 1, n_sel);
        if isfield(est, 'tracks') && ~isempty(est.tracks)
            Kt = min(numel(frame_times), numel(est.tracks));
            for k = 1:Kt
                Tr = est.tracks{k};
                if isempty(Tr) || ~isfield(Tr, 'L') || isempty(Tr.L) || ...
                        ~isfield(Tr, 'm') || isempty(Tr.m)
                    continue;
                end
                for c = 1:size(Tr.L, 2)
                    tid = Tr.L(2, c);
                    if isKey(idmap, tid) && size(Tr.m, 1) >= 7 && c <= size(Tr.m, 2)
                        s = idmap(tid);
                        FILT(s).t(end+1) = frame_times(k); %#ok<AGROW>
                        FILT(s).E(end+1) = Tr.m(1, c);     %#ok<AGROW>
                        FILT(s).N(end+1) = Tr.m(4, c);     %#ok<AGROW>
                        FILT(s).U(end+1) = Tr.m(7, c);     %#ok<AGROW>
                    end
                end
            end
        end
        for s = 1:n_sel
            if ~isempty(FILT(s).t)
                [FILT(s).t, ord] = sort(FILT(s).t);
                FILT(s).E = FILT(s).E(ord);
                FILT(s).N = FILT(s).N(ord);
                FILT(s).U = FILT(s).U(ord);
            end
        end

        % -- 伪真值：复用 plot_pseudo_truth 的提取逻辑，只返回数据不另开3D图 --
        to = struct();
        to.track_ids = sel_ids;
        to.no_figure = true;
        to.tid_select_mode = TRUTH_SELECT_MODE;
        to.truth_all = TRUTH_ALL;
        if have_cfg
            to.cfg = cfg;
        end
        if ~isempty(TRUTH_USE_SPLIT), to.truth_use_split = TRUTH_USE_SPLIT; end
        if have_full_truth
            to.fused_xyz = fused_xyz;
            to.fused_ids = fused_ids;
        end
        info_truth = plot_pseudo_truth(est, frame_times, to);
        truth_tracks = repmat(struct('tid', NaN, 't', [], 'p', zeros(3, 0)), 1, 0);
        if isstruct(info_truth) && isfield(info_truth, 'TRk')
            truth_tracks = info_truth.TRk;
        end
        truth_use_split = isstruct(info_truth) && isfield(info_truth, 'truth_use_split') && info_truth.truth_use_split;
        truth_split_info = [];
        if isstruct(info_truth) && isfield(info_truth, 'truth_split_info')
            truth_split_info = info_truth.truth_split_info;
        end

        % -- 建立选中航迹ID到主伪真值绘图编号的对应关系，用于误差图配对 --
        track_tid = nan(1, n_sel);
        if isstruct(info_truth) && isfield(info_truth, 'track_tid_summary') && ...
                ~isempty(info_truth.track_tid_summary)
            ss = info_truth.track_tid_summary;
            for q = 1:numel(ss)
                if ~isfield(ss(q), 'track_id') || ~isfield(ss(q), 'main_tid'), continue; end
                if isKey(idmap, ss(q).track_id)
                    s = idmap(ss(q).track_id);
                    track_tid(s) = ss(q).main_tid;
                end
            end
        end
        if any(~isfinite(track_tid)) && isfield(est, 'assoc') && ~isempty(est.assoc) && ~truth_use_split
            tid_votes = cell(1, n_sel);
            for s = 1:n_sel, tid_votes{s} = zeros(1, 0); end
            for k = 1:numel(frame_times)
                if k > numel(est.assoc), continue; end
                As = est.assoc{k};
                if isempty(As) || ~isfield(As, 'id') || isempty(As.id) || ...
                        ~isfield(As, 'tid') || isempty(As.tid)
                    continue;
                end
                ids = As.id(:).';
                tids = As.tid(:).';
                n = min(numel(ids), numel(tids));
                for c = 1:n
                    if ~isfinite(tids(c)), continue; end
                    if isKey(idmap, ids(c))
                        s = idmap(ids(c));
                        tid_votes{s}(end+1) = tids(c); %#ok<AGROW>
                    end
                end
            end
            for s = 1:n_sel
                if ~isempty(tid_votes{s})
                    u = unique(tid_votes{s});
                    cnt = zeros(size(u));
                    for ii = 1:numel(u), cnt(ii) = sum(tid_votes{s} == u(ii)); end
                    [~, im] = max(cnt);
                    track_tid(s) = u(im);
                end
            end
        end

        truth_by_tid = containers.Map('KeyType', 'double', 'ValueType', 'double');
        for q = 1:numel(truth_tracks)
            if isfinite(truth_tracks(q).tid)
                truth_by_tid(truth_tracks(q).tid) = q;
            end
        end

        ft_unique = unique(frame_times(:));
        if numel(ft_unique) >= 2
            frame_dt_ref = median(diff(ft_unique));
        else
            frame_dt_ref = 0;
        end

        % -- 配色 --
        base = lines(max(n_sel, 1));
        hsvc = rgb2hsv(base(1:n_sel, :));
        hsvc(:, 2) = max(hsvc(:, 2), 0.75);
        hsvc(:, 3) = max(hsvc(:, 3), 0.80);
        filt_cols = hsv2rgb(hsvc);

        truth_cols = repmat([0.25 0.25 0.25], max(numel(truth_tracks), 1), 1);
        used_truth_color = false(1, max(numel(truth_tracks), 1));
        for s = 1:n_sel
            if isfinite(track_tid(s)) && isKey(truth_by_tid, track_tid(s))
                q = truth_by_tid(track_tid(s));
                truth_cols(q, :) = 0.55 * filt_cols(s, :) + 0.45 * [1 1 1];
                used_truth_color(q) = true;
            end
        end
        fallback_cols = lines(max(numel(truth_tracks), 1));
        for q = 1:numel(truth_tracks)
            if ~used_truth_color(q)
                truth_cols(q, :) = 0.45 * fallback_cols(q, :) + 0.55 * [1 1 1];
            end
        end

        % -- 画三轴时间序列：滤波=粗实线，伪真值=细实线+小圆点 --
        figure('Name', '滤波估值 vs 伪真值（ENU 时间序列）', ...
               'Color', 'w', 'Position', [80, 80, 1100, 900]);
        for ax_i = 1:3
            subplot(3, 1, ax_i); hold on; grid on; box on;

            for s = 1:n_sel
                if ~isempty(FILT(s).t)
                    plot(FILT(s).t, FILT(s).(fields{ax_i}), '-', ...
                         'Color', filt_cols(s, :), 'LineWidth', 2.0, ...
                         'DisplayName', sprintf('滤波 ID=%g', sel_ids(s)));
                end
            end

            for q = 1:numel(truth_tracks)
                tk = truth_tracks(q);
                if isempty(tk.t) || isempty(tk.p) || size(tk.p, 1) < 3
                    continue;
                end
                plot(tk.t, tk.p(ax_i, :), 'o-', ...
                     'Color', truth_cols(q, :), 'LineWidth', 1.0, ...
                     'MarkerSize', 3, 'MarkerFaceColor', 'w', ...
                     'DisplayName', local_truth_display_name(tk.tid, truth_split_info, truth_use_split));
            end

            xlabel('时间 (s)'); ylabel(ax_lab{ax_i});
            if ax_i == 1
                legend('Location', 'best');
            end
        end
        sgtitle(sprintf('滤波估值 vs 伪真值 三轴时间序列 (航迹: %s)', mat2str(sel_ids)));

        % -- 画三轴误差：滤波值 - 插值到滤波时刻的伪真值 --
        figure('Name', '滤波估值 - 伪真值（ENU 三轴误差）', ...
               'Color', 'w', 'Position', [130, 100, 1100, 900]);
        for ax_i = 1:3
            subplot(3, 1, ax_i); hold on; grid on; box on;
            for s = 1:n_sel
                if isempty(FILT(s).t) || ~isfinite(track_tid(s)) || ~isKey(truth_by_tid, track_tid(s))
                    continue;
                end
                tk = truth_tracks(truth_by_tid(track_tid(s)));
                if isempty(tk.t) || numel(tk.t) < 1 || size(tk.p, 1) < 3
                    continue;
                end
                [tt, uidx] = unique(tk.t(:).', 'stable');
                yy = tk.p(ax_i, uidx);
                ft = FILT(s).t(:).';
                fv = FILT(s).(fields{ax_i})(:).';
                if numel(tt) >= 2
                    mask = ft >= tt(1) & ft <= tt(end);
                    tv = nan(size(ft));
                    tv(mask) = interp1(tt, yy, ft(mask), 'linear');
                else
                    mask = abs(ft - tt(1)) <= max(eps, 0.5 * frame_dt_ref);
                    tv = nan(size(ft));
                    tv(mask) = yy(1);
                end
                ok = isfinite(fv) & isfinite(tv);
                if ~any(ok), continue; end
                plot(ft(ok), fv(ok) - tv(ok), '-', ...
                     'Color', filt_cols(s, :), 'LineWidth', 1.7, ...
                     'DisplayName', sprintf('ID=%g - %s', sel_ids(s), ...
                         local_truth_display_name(tk.tid, truth_split_info, truth_use_split)));
            end
            xl = xlim;
            plot(xl, [0 0], 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
            xlim(xl);
            xlabel('时间 (s)'); ylabel(sprintf('\\Delta%s (m)', fields{ax_i}));
            if ax_i == 1
                legend('Location', 'best');
            end
        end
        sgtitle(sprintf('滤波估值 - 伪真值 三轴误差 (航迹: %s)', mat2str(sel_ids)));
    end
end

function name = local_truth_display_name(key, split_info, use_split)
if use_split && isstruct(split_info) && isfield(split_info, 'key_raw_id') && ...
        key >= 1 && key <= numel(split_info.key_raw_id) && isfinite(split_info.key_raw_id(key))
    name = sprintf('伪真值 key=%g(原tid=%g,实例=%g)', ...
        key, split_info.key_raw_id(key), split_info.key_instance(key));
else
    name = sprintf('伪真值 tid=%g', key);
end
end
