function id = joint_legacy_committed_logical_id(external_id, companions, offset)
%JOINT_LEGACY_COMMITTED_LOGICAL_ID Resolve only committed branch mappings.
% A pending_external_3d_id is deliberately ignored until atomic commit.

if nargin < 3 || isempty(offset), offset = 1000000; end
id = external_id;
if isempty(companions) || ~isfinite(external_id), return; end
if ~isfield(companions, 'external_3d_id'), return; end
i = find([companions.external_3d_id] == external_id, 1);
if isempty(i), return; end
q = companions(i);
from_2d = isfield(q, 'from_2d') && logical(q.from_2d);
if from_2d && isfield(q, 'local_2d_id') && isfinite(q.local_2d_id)
    id = q.local_2d_id + offset;
elseif isfield(q, 'logical_id') && isfinite(q.logical_id) && q.logical_id > 0
    id = q.logical_id;
end
end
