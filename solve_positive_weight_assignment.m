function [pairs, info] = solve_positive_weight_assignment(weight)
%SOLVE_POSITIVE_WEIGHT_ASSIGNMENT Maximum-weight matching on positive edges.
%   Zero/non-finite entries mean that no useful pairing exists. Independent
%   bipartite components are solved separately so unrelated truth/track IDs
%   never inflate one global Hungarian problem.

[n_row, n_col] = size(weight);
info = struct('original_size', [n_row, n_col], 'active_size', [0, 0], ...
    'n_components', 0, 'max_component_size', [0, 0], ...
    'n_pairs', 0, 'objective', 0, 'elapsed_s', 0, ...
    'n_builtin_components', 0, 'n_fallback_components', 0);
pairs = zeros(0, 2);
t_start = tic;

if isempty(weight) || ~isnumeric(weight)
    info.elapsed_s = toc(t_start);
    return;
end

weight = double(weight);
weight(~isfinite(weight) | weight <= 0) = 0;
active_row = find(any(weight > 0, 2));
active_col = find(any(weight > 0, 1));
info.active_size = [numel(active_row), numel(active_col)];
if isempty(active_row) || isempty(active_col)
    info.elapsed_s = toc(t_start);
    return;
end

positive = sparse(weight(active_row, active_col) > 0);
components = bipartite_components(positive);
info.n_components = numel(components);

for k = 1:numel(components)
    local_rows = components(k).rows;
    local_cols = components(k).cols;
    W = weight(active_row(local_rows), active_col(local_cols));
    info.max_component_size = max(info.max_component_size, size(W));

    [local_pairs, backend] = solve_component(W);
    if strcmp(backend, 'builtin')
        info.n_builtin_components = info.n_builtin_components + 1;
    else
        info.n_fallback_components = info.n_fallback_components + 1;
    end
    if isempty(local_pairs)
        continue;
    end
    mapped_row = active_row(local_rows(local_pairs(:, 1)));
    mapped_col = active_col(local_cols(local_pairs(:, 2)));
    mapped = [mapped_row(:), mapped_col(:)];
    pairs = [pairs; mapped]; %#ok<AGROW>
end

if ~isempty(pairs)
    [~, order] = sortrows(pairs, [1, 2]);
    pairs = pairs(order, :);
    linear = sub2ind([n_row, n_col], pairs(:, 1), pairs(:, 2));
    info.objective = sum(weight(linear));
end
info.n_pairs = size(pairs, 1);
info.elapsed_s = toc(t_start);
end

function components = bipartite_components(edge)
n_row = size(edge, 1);
seen_row = false(n_row, 1);
components = repmat(struct('rows', zeros(1, 0), ...
    'cols', zeros(1, 0)), n_row, 1);
n_component = 0;

for seed = 1:n_row
    if seen_row(seed)
        continue;
    end
    row_mask = false(n_row, 1);
    row_mask(seed) = true;
    col_mask = false(1, size(edge, 2));
    while true
        next_col = any(edge(row_mask, :), 1);
        next_row = any(edge(:, next_col), 2);
        if isequal(next_col, col_mask) && isequal(next_row, row_mask)
            break;
        end
        col_mask = next_col;
        row_mask = next_row;
    end
    seen_row(row_mask) = true;
    n_component = n_component + 1;
    components(n_component, 1) = struct( ...
        'rows', find(row_mask).', 'cols', find(col_mask));
end
components = components(1:n_component);
end

function [pairs, backend] = solve_component(weight)
[n_row, n_col] = size(weight);
original_weight = weight;
pairs = zeros(0, 2);
backend = 'direct';
if n_row == 0 || n_col == 0
    return;
end
if n_row == 1
    [best, col] = max(weight(1, :));
    if best > 0, pairs = [1, col]; end
    return;
end
if n_col == 1
    [best, row] = max(weight(:, 1));
    if best > 0, pairs = [row, 1]; end
    return;
end

% matchpairs is compiled and handles optional unmatched rows/columns. A
% zero unmatched cost with negative edge costs is exactly maximum-weight
% matching over positive edges.
if exist('matchpairs', 'file') == 2
    cost = -weight;
    cost(weight <= 0) = inf;
    pairs = matchpairs(cost, 0);
    backend = 'builtin';
else
    transposed = n_row > n_col;
    if transposed
        weight = weight.';
    end
    max_weight = max(weight(:));
    cost = max_weight - weight;
    cost(weight <= 0) = inf;
    pairs = solve_global_assignment(cost, max_weight);
    if transposed && ~isempty(pairs)
        pairs = pairs(:, [2, 1]);
    end
    backend = 'fallback';
end

if ~isempty(pairs)
    good = original_weight(sub2ind(size(original_weight), ...
        pairs(:, 1), pairs(:, 2))) > 0;
    pairs = pairs(good, :);
end
end
