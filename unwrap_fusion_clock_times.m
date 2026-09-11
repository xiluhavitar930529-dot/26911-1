function unwrapped = unwrap_fusion_clock_times(times)
%UNWRAP_FUSION_CLOCK_TIMES Expand midnight rollovers in acquisition order.

unwrapped = times;
offset = 0;
previous = NaN;
for i = 1:numel(times)
    if ~isfinite(times(i)), continue; end
    candidate = times(i) + offset;
    if isfinite(previous) && candidate < previous - 12 * 3600
        offset = offset + 24 * 3600;
        candidate = times(i) + offset;
    end
    unwrapped(i) = candidate;
    previous = candidate;
end
end
