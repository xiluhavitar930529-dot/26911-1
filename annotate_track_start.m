function h = annotate_track_start(ax, coordinates, track_id, color)
%ANNOTATE_TRACK_START Label the first finite point of a 2-D or 3-D track.

if nargin < 1 || isempty(ax), ax = gca; end
if nargin < 2 || isempty(coordinates), h = gobjects(1, 0); return; end
if nargin < 3 || ~isscalar(track_id) || ~isfinite(track_id)
    h = gobjects(1, 0);
    return;
end
if nargin < 4 || isempty(color), color = [0 0 0]; end

dimension = size(coordinates, 1);
if dimension ~= 2 && dimension ~= 3
    error('annotate_track_start:InvalidDimension', ...
        '航迹坐标必须是2xN或3xN矩阵。');
end
first = find(all(isfinite(coordinates), 1), 1);
if isempty(first), h = gobjects(1, 0); return; end

label = sprintf(' Track %g', track_id);
common = {'Color', color, 'FontSize', 9, 'FontWeight', 'bold', ...
    'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ...
    'Interpreter', 'none', 'Clipping', 'on', 'HandleVisibility', 'off'};
if dimension == 2
    h = text(ax, coordinates(1, first), coordinates(2, first), label, common{:});
else
    h = text(ax, coordinates(1, first), coordinates(2, first), ...
        coordinates(3, first), label, common{:});
end
end
