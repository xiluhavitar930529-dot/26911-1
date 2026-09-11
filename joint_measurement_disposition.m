function d = joint_measurement_disposition(n, a, type, suppressed, event_index)
%JOINT_MEASUREMENT_DISPOSITION Reconcile every filter input exactly once.
% Array position is the measurement index in events(k).active/passive.
% action: 1=associated, 2=birth, 3=suppressed because an existing track
% explains the measurement. Confirmation/formal output is not required.
% input_dim is the branch before lifecycle/capacity pruning; filter_dim is
% the retained branch. Suppressed inputs did not enter either branch (0).
assert(numel(suppressed) == n && islogical(suppressed), ...
    'run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
    'Suppression mask must identify the input measurements.');
d = struct('action', zeros(1, n, 'uint8'), 'track_id', zeros(1, n), ...
    'filter_dim', zeros(1, n, 'uint8'), 'input_dim', zeros(1, n, 'uint8'), ...
    'range_updated', false(1, n));
d.action(suppressed) = 3;
q = find(strncmp(a.type, type, numel(type)));
mi = a.meas_index(q);
if any(~isfinite(mi) | mi < 1 | mi > n | mi ~= round(mi)) || ...
        numel(unique(mi)) ~= numel(mi)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s has duplicate or invalid measurement indices.', event_index, type);
end
if any(d.action(mi) ~= 0)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s is both associated and suppressed.', event_index, type);
end
d.action(mi) = uint8(1 + strcmp(a.type(q), [type '_birth']));
d.track_id(mi) = a.id(q);
d.filter_dim(mi) = uint8(a.filter_dim(q));
d.input_dim(mi) = d.filter_dim(mi);
if isfield(a, 'input_dim'), d.input_dim(mi) = uint8(a.input_dim(q)); end
d.range_updated(mi) = a.range_updated(q);
if any(d.action == 0)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s contains measurements without a disposition.', event_index, type);
end
end
