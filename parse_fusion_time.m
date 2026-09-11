function t_sec = parse_fusion_time(value, format)
%PARSE_FUSION_TIME Parse configured clock or numeric timestamps to seconds.

if nargin < 2 || isempty(format), format = 'seconds'; end
text = strtrim(char(string(value)));
text = strrep(text, char(65279), '');
if isempty(text), t_sec = NaN; return; end

switch lower(strtrim(char(format)))
    case 'hms'
        if contains(text, ':')
            fields = strsplit(text, ':');
            if numel(fields) ~= 3, t_sec = NaN; return; end
            h = str2double(fields{1});
            m = str2double(fields{2});
            s = str2double(fields{3});
        else
            token = regexp(text, '^([0-9]{1,6})([.][0-9]+)?$', ...
                'tokens', 'once');
            if isempty(token), t_sec = NaN; return; end
            whole = token{1};
            whole = [repmat('0', 1, 6 - numel(whole)), whole];
            h = str2double(whole(1:2));
            m = str2double(whole(3:4));
            s = str2double(whole(5:6));
            if numel(token) >= 2 && ~isempty(token{2})
                s = s + str2double(token{2});
            end
        end
        if any(~isfinite([h, m, s])) || h < 0 || h >= 24 || ...
                m < 0 || m >= 60 || s < 0 || s >= 60
            t_sec = NaN;
        else
            t_sec = h * 3600 + m * 60 + s;
        end
    case {'seconds', 'second', 'sec', 's', 'numeric'}
        t_sec = str2double(text);
    otherwise
        t_sec = NaN;
end
end
