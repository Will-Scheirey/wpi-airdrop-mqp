function drop_num = get_drop_num(dirname)
    tokens = regexp(dirname, 'DN(\d+)', 'tokens');

    if ~isempty(tokens)
        drop_num = str2double(tokens{1}{1});
    else
        drop_num = NaN;
    end
end