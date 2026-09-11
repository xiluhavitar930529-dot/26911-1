function tk = get_smoothed_track(smt, id)
%GET_SMOOTHED_TRACK  从 smooth_filter_tracks 的输出中按航迹ID取平滑结果。
%   返回 struct(id,t,raw,smooth,n,rms_shift)；找不到返回空结构([])。
tk = [];
if isempty(smt) || ~isfield(smt, 'tracks') || isempty(smt.tracks), return; end
ids = [smt.tracks.id];
j = find(ids == id, 1);
if ~isempty(j), tk = smt.tracks(j); end
end
