function [err, t_common] = interpolate_and_error(t1, d1, t2, d2)
% Align and interpolate two time series (t1, d1) and (t2, d2), then compute the error (N×M).
% The longer dataset is used as the interpolation reference.
% Returns:
%   err       — Element-wise error after interpolation (N×M)
%   t_common  — Common aligned time axis (column vector)

    if length(t1) >= length(t2)
        t_common = t2(:);  % using t2 
        d1_interp = interp1(t1, d1, t_common, 'linear', 'extrap');
        err = d2 - d1_interp;
    else
        t_common = t1(:);  % using t1  
        d2_interp = interp1(t2, d2, t_common, 'linear', 'extrap');
        err = d2_interp - d1;
    end
end