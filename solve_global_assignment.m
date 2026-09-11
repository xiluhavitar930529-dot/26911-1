function pairs = solve_global_assignment(cost, unmatched_cost)
%SOLVE_GLOBAL_ASSIGNMENT Toolbox-independent rectangular Hungarian solver.
%
% Every row is assigned either to one real measurement column or to one of
% the added dummy columns with unmatched_cost. Real measurement columns are
% unique; unused measurements have no penalty.

[n, m] = size(cost);
pairs = zeros(0, 2);
if n == 0 || m == 0 || ~any(isfinite(cost(:)))
    return;
end
if ~isscalar(unmatched_cost) || ~isfinite(unmatched_cost)
    error('solve_global_assignment:InvalidUnmatchedCost', ...
        'unmatched_cost must be a finite scalar.');
end

large = max(1, abs(unmatched_cost)) * 1e9;
C = cost;
C(~isfinite(C)) = large;
C = [C, unmatched_cost * ones(n, n)];
ncol = size(C, 2);

% Shortest augmenting path form of the Hungarian algorithm (n <= ncol).
u = zeros(n + 1, 1);
v = zeros(ncol + 1, 1);
p = zeros(ncol + 1, 1);
way = zeros(ncol + 1, 1);
for i = 1:n
    p(1) = i;
    j0 = 1;
    minv = inf(ncol + 1, 1);
    used = false(ncol + 1, 1);
    while true
        used(j0) = true;
        i0 = p(j0);
        delta = inf;
        j1 = 0;
        for j = 2:ncol + 1
            if used(j), continue; end
            cur = C(i0, j - 1) - u(i0 + 1) - v(j);
            if cur < minv(j)
                minv(j) = cur;
                way(j) = j0;
            end
            if minv(j) < delta
                delta = minv(j);
                j1 = j;
            end
        end
        if ~isfinite(delta) || j1 == 0
            error('solve_global_assignment:NoAugmentingPath', ...
                'Failed to construct a finite assignment.');
        end
        for j = 1:ncol + 1
            if used(j)
                u(p(j) + 1) = u(p(j) + 1) + delta;
                v(j) = v(j) - delta;
            else
                minv(j) = minv(j) - delta;
            end
        end
        j0 = j1;
        if p(j0) == 0, break; end
    end
    while true
        j1 = way(j0);
        p(j0) = p(j1);
        j0 = j1;
        if j0 == 1, break; end
    end
end

assignment = zeros(n, 1);
for j = 2:ncol + 1
    if p(j) > 0
        assignment(p(j)) = j - 1;
    end
end
for i = 1:n
    j = assignment(i);
    if j >= 1 && j <= m && isfinite(cost(i, j)) && cost(i, j) <= unmatched_cost
        pairs(end + 1, :) = [i, j]; %#ok<AGROW>
    end
end
end
