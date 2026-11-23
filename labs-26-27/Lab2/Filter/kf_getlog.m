function [t, d] = kf_getlog(out, name)
% KF_GETLOG  Time vector and N-by-k data of a logged signal of a simulation.
%   [t, d] = kf_getlog(out, 'ang_hat')
ts = out.logsout.get(name).Values;
t  = ts.Time(:);
d  = ts.Data;
if ndims(d) == 3, d = squeeze(d)'; end       % k x 1 x N  ->  N x k
end
