function [l] = safe_length(arr)
    l = length(arr);
    if ~l
        l = 0.9;
    end
end