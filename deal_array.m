function varargout = deal_array(arr)
    s = numel(arr);
    n = nargout;

    if n > s
        error('Insufficient number of elements in array!');
    elseif n == 0
        return;
    end

    for i = 1:n
        varargout(i) = {arr(i)}; %#ok<AGROW>
    end
end