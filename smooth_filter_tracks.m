function smt = smooth_filter_tracks(est, frame_times, opts)
%SMOOTH_FILTER_TRACKS  对前向滤波得到的确认航迹做"二次平滑滤波"。
%   默认为【在线(因果)平滑】：固定滞后平滑器(fixed-lag smoother)。
%   把前向滤波输出的每条航迹位置序列当作量测，再跑一遍二次平滑：
%     'fixedlag' 在线固定滞后平滑(默认)：点j的平滑值只依赖到 j+lag 为止的数据，
%                完全因果、可实时；lag=0 退化为纯前向二次CKF(零滞后)。
%     'forward'  在线零滞后(=fixedlag, lag=0)：纯前向二次CKF，无延迟但平滑较弱。
%     'movmean'  在线尾部滑动平均(因果)：仅用过去 win 帧。
%     'rts'      ★离线★ 固定区间RTS(用到未来全部数据)，仅供对比，非实时可用。
%
%   "在线"含义：每个输出点 j 的结果只用到不晚于 j+lag 的量测，绝不使用更远的未来，
%   因此可在实时流中以"滞后 lag 帧"的延迟逐帧产出，等价于真实系统的固定滞后平滑。
%
%   输入：
%     est, frame_times   —— 同 run_filter_adapt_ckf / play_track_filter 的输出
%     opts（全部可选）:
%       method     'fixedlag'|'forward'|'movmean'|'rts'   默认 'fixedlag'
%       lag        固定滞后帧数(fixedlag)                  默认 8
%       track_ids  指定航迹ID；留空=全部确认航迹           默认 []
%       min_life   最短帧数(短于此不平滑)                  默认 10
%       meas_std   量测(=滤波位置)噪声std (m)              默认 30
%       proc_std   过程加速度std (m/s^2)                   默认 3
%       win        movmean: 尾部窗口(帧)                   默认 7
%       pos_idx    位置在状态中的下标                      默认 [1 4 7]
%       verbose    打印每条航迹平滑量                      默认 true
%
%   输出 smt:
%     .ids        1×T 被平滑的航迹ID
%     .tracks     1×T struct: id,t[1×m],raw[3×m],smooth[3×m],n,rms_shift
%     .method, .params(含 .lag, .online)
%
%   典型用法：
%     smt = smooth_filter_tracks(est, frame_times);                 % 在线,滞后8帧
%     o.method='forward'; smt = smooth_filter_tracks(est,frame_times,o); % 零滞后
%     plot_track_smoothing(smt);                       % 单独出平滑前/后对比图
%     p.smooth = smt; play_track_filter(est, frame_times, p);  % 与回放一块出

if nargin < 3 || isempty(opts), opts = struct(); end
gp = @(f, d) smt_get(opts, f, d);

method    = lower(gp('method', 'fixedlag'));
lag       = max(0, round(gp('lag', 8)));
track_ids = gp('track_ids', []);
min_life  = max(1, round(gp('min_life', 10)));
meas_std  = gp('meas_std', 30);
proc_std  = gp('proc_std', 3);
win       = gp('win', 7); win = max(2, round(win));
pos_idx   = gp('pos_idx', [1, 4, 7]);
verbose   = gp('verbose', true) ~= 0;

if strcmp(method, 'forward'), method = 'fixedlag'; lag = 0; end
online = ~strcmp(method, 'rts');

K = numel(frame_times);
if K == 0 || ~isfield(est, 'L') || ~isfield(est, 'X')
    warning('est 缺少 L/X 字段或帧数为0，无法平滑。');
    smt = empty_smt(method, meas_std, proc_std, win, lag, online); return;
end

%% ── 提取每条确认航迹的位置时间序列（按ID聚合）─────────────────────────
raw = collect_tracks(est, frame_times, pos_idx, K);
if isempty(raw)
    warning('未提取到任何确认航迹位置序列。');
    smt = empty_smt(method, meas_std, proc_std, win, lag, online); return;
end

all_ids = [raw.id];
if isempty(track_ids), sel = true(1, numel(raw));
else, sel = ismember(all_ids, track_ids(:).'); end
sel = sel & ([raw.n] >= min_life);
raw = raw(sel);
if isempty(raw)
    warning('按 track_ids/min_life 过滤后无可平滑航迹。');
    smt = empty_smt(method, meas_std, proc_std, win, lag, online); return;
end

%% ── 逐条平滑 ──────────────────────────────────────────────────────────
T = numel(raw);
tracks = repmat(struct('id', NaN, 't', [], 'raw', zeros(3, 0), ...
    'smooth', zeros(3, 0), 'n', 0, 'rms_shift', 0), 1, T);
for i = 1:T
    t = raw(i).t; P = raw(i).p; n = raw(i).n;
    S = P;
    if n >= 2
        switch method
            case 'fixedlag'   % 在线固定滞后(lag=0即纯前向)
                for ax = 1:3
                    S(ax, :) = cv_fixedlag_axis(t, P(ax, :), meas_std, proc_std, lag);
                end
            case 'movmean'    % 在线尾部滑动平均(因果)
                for ax = 1:3
                    S(ax, :) = trailing_movmean(P(ax, :), win);
                end
            case 'rts'        % 离线固定区间RTS(用未来全部数据)
                for ax = 1:3
                    S(ax, :) = cv_rts_axis(t, P(ax, :), meas_std, proc_std);
                end
            otherwise
                error('未知 method=%s（fixedlag/forward/movmean/rts）', method);
        end
    end
    d = S - P;
    rms_shift = sqrt(mean(sum(d.^2, 1)));
    tracks(i) = struct('id', raw(i).id, 't', t, 'raw', P, ...
        'smooth', S, 'n', n, 'rms_shift', rms_shift);
end

smt = struct();
smt.ids = [tracks.id];
smt.tracks = tracks;
smt.method = method;
smt.params = struct('meas_std', meas_std, 'proc_std', proc_std, ...
    'win', win, 'lag', lag, 'online', online, ...
    'min_life', min_life, 'pos_idx', pos_idx);

if verbose
    if online
        if strcmp(method, 'movmean')
            mode_txt = sprintf('在线/尾部均值 win=%d帧', win);
        elseif lag == 0
            mode_txt = '在线/零滞后(纯前向二次CKF)';
        else
            mode_txt = sprintf('在线/固定滞后 lag=%d帧', lag);
        end
    else
        mode_txt = '★离线★ 固定区间RTS(用未来全部数据)';
    end
    fprintf('[二次平滑] method=%s  %s  航迹%d条\n', method, mode_txt, T);
    if ~strcmp(method, 'movmean')
        fprintf('          meas_std=%.1fm, proc_std=%.2f m/s^2\n', meas_std, proc_std);
    end
    [~, ord] = sort([tracks.rms_shift], 'descend');
    np = min(T, 12);
    for ii = 1:np
        s = tracks(ord(ii));
        fprintf('          est%-5g  帧=%-4d  平滑位移RMS=%.2f m\n', s.id, s.n, s.rms_shift);
    end
    if T > np, fprintf('          ...(其余%d条略)\n', T - np); end
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function raw = collect_tracks(est, frame_times, pos_idx, K)
% 按航迹ID聚合 (时间, 位置)。同一帧同一ID只取首条。
id_list = [];
cellT = {}; cellP = {};
idmap = containers.Map('KeyType', 'double', 'ValueType', 'double');
for k = 1:K
    if isempty(est.L{k}), continue; end
    Lk = est.L{k};
    Xk = est.X{k};
    nrow = size(Lk, 1);
    for i = 1:nrow
        tid = Lk(i, 2);
        if i > size(Xk, 2), continue; end
        pos = Xk(pos_idx, i);
        if ~all(isfinite(pos)), continue; end
        if isKey(idmap, tid)
            s = idmap(tid);
        else
            s = numel(id_list) + 1;
            idmap(tid) = s; id_list(s) = tid; %#ok<AGROW>
            cellT{s} = zeros(1, 0); cellP{s} = zeros(3, 0); %#ok<AGROW>
        end
        cellT{s}(end+1) = frame_times(k);       %#ok<AGROW>
        cellP{s}(:, end+1) = pos(:);            %#ok<AGROW>
    end
end
T = numel(id_list);
raw = repmat(struct('id', NaN, 't', [], 'p', zeros(3, 0), 'n', 0), 1, T);
for s = 1:T
    [ts, ord] = sort(cellT{s});
    ps = cellP{s}(:, ord);
    raw(s) = struct('id', id_list(s), 't', ts, 'p', ps, 'n', numel(ts));
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function [xf, Pf, xp, Pp] = cv_forward(t, z, rstd, qstd)
% 单轴 定速(CV)模型 前向CKF（因果），导出先验/后验均值与协方差。
n = numel(z);
R = max(rstd, 1e-6)^2;
q = max(qstd, 1e-9)^2;
xf = zeros(2, n); Pf = zeros(2, 2, n);
xp = zeros(2, n); Pp = zeros(2, 2, n);
x = [z(1); 0];
P = diag([R, (10 * max(rstd, 1))^2]);
xp(:, 1) = x; Pp(:, :, 1) = P;
[x, P] = ckf_smooth_update_scalar(z(1), R, x, P);
xf(:, 1) = x; Pf(:, :, 1) = P;
for k = 2:n
    dt = max(t(k) - t(k-1), 1e-3);
    F = [1, dt; 0, 1];
    Q = q * [dt^3/3, dt^2/2; dt^2/2, dt];
    [x, P] = ckf_smooth_predict(x, P, F, Q);
    xp(:, k) = x; Pp(:, :, k) = P;
    [x, P] = ckf_smooth_update_scalar(z(k), R, x, P);
    xf(:, k) = x; Pf(:, :, k) = P;
end
end

function [x_pred, P_pred] = ckf_smooth_predict(x, P, F, Q)
[Xi, w] = ckf_smooth_points(x, P);
Xp = F * Xi;
x_pred = sum(Xp, 2) * w;
dx = bsxfun(@minus, Xp, x_pred);
P_pred = smt_make_spd(dx * dx' * w + Q);
end

function [x_upd, P_upd] = ckf_smooth_update_scalar(z, R, x, P)
[Xi, w] = ckf_smooth_points(x, P);
Zi = Xi(1, :);
z_pred = sum(Zi) * w;
dx = bsxfun(@minus, Xi, x(:));
dz = Zi - z_pred;
Pzz = dz * dz' * w + R;
Pxz = dx * dz' * w;
K = Pxz / Pzz;
x_upd = x + K * (z - z_pred);
P_upd = smt_make_spd(P - K * Pzz * K');
end

function [Xi, w] = ckf_smooth_points(x, P)
x = x(:);
n = numel(x);
P = smt_make_spd(P);
[S, flag] = chol(P, 'lower');
if flag ~= 0
    [V, D] = eig(P);
    S = V * diag(sqrt(max(diag(D), 1e-9)));
end
scale = sqrt(n);
Xi = [bsxfun(@plus, x, scale * S), bsxfun(@minus, x, scale * S)];
w = 1 / (2 * n);
end

function A = smt_make_spd(A)
A = 0.5 * (A + A');
[V, D] = eig(A);
d = max(diag(D), 1e-9);
A = V * diag(d) * V';
A = 0.5 * (A + A');
end

%% ═══════════════════════════════════════════════════════════════════════════
function xs = cv_fixedlag_axis(t, z, rstd, qstd, L)
% 在线固定滞后平滑：输出点j = x_{j | min(j+L,n)}，仅用到 j+L 为止的量测(因果)。
n = numel(z);
if n == 1, xs = z; return; end
[xf, Pf, xp, Pp] = cv_forward(t, z, rstd, qstd);
if L == 0, xs = xf(1, :); return; end   % 零滞后=纯前向
xs = zeros(1, n);
for j = 1:n
    e = min(j + L, n);
    s = xf(:, e);
    for k = e-1:-1:j
        dt = max(t(k+1) - t(k), 1e-3);
        F = [1, dt; 0, 1];
        C = (Pf(:, :, k) * F') / Pp(:, :, k+1);
        s = xf(:, k) + C * (s - xp(:, k+1));
    end
    xs(j) = s(1);
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function xs = cv_rts_axis(t, z, rstd, qstd)
% ★离线★ 单轴 定速模型 固定区间RTS(前向CKF+后向RTS，使用未来全部数据)。
n = numel(z);
if n == 1, xs = z; return; end
[xf, Pf, xp, Pp] = cv_forward(t, z, rstd, qstd);
xsm = xf;
for k = n-1:-1:1
    dt = max(t(k+1) - t(k), 1e-3);
    F = [1, dt; 0, 1];
    C = (Pf(:, :, k) * F') / Pp(:, :, k+1);
    xsm(:, k) = xf(:, k) + C * (xsm(:, k+1) - xp(:, k+1));
end
xs = xsm(1, :);
end

%% ═══════════════════════════════════════════════════════════════════════════
function y = trailing_movmean(x, w)
% 在线尾部滑动平均（仅用过去 w 帧，因果）。
n = numel(x); y = x;
for i = 1:n
    a = max(1, i - w + 1);
    y(i) = mean(x(a:i));
end
end

%% ═══════════════════════════════════════════════════════════════════════════
function smt = empty_smt(method, meas_std, proc_std, win, lag, online)
smt = struct('ids', [], ...
    'tracks', repmat(struct('id', NaN, 't', [], 'raw', zeros(3, 0), ...
        'smooth', zeros(3, 0), 'n', 0, 'rms_shift', 0), 1, 0), ...
    'method', method, ...
    'params', struct('meas_std', meas_std, 'proc_std', proc_std, ...
        'win', win, 'lag', lag, 'online', online));
end

%% ═══════════════════════════════════════════════════════════════════════════
function v = smt_get(opts, f, d)
if isfield(opts, f) && ~isempty(opts.(f)), v = opts.(f); else, v = d; end
end
