function make_report_figures(opts)
% MAKE_REPORT_FIGURES  Run every experiment of the report and export its figures.
%
%   make_report_figures                 % everything, ~40 simulations
%   make_report_figures(sweeps=false)   % skip the two sensitivity sweeps (fast)
%
% Writes PNGs (150 dpi, same style as kalman_filters.m) to report/overleaf/figs/, next to report/overleaf/main.tex and the Simulink
% diagrams for the appendix. All data are also saved in report/report_data.mat.
% The models must exist (run build_batista_kf_model once if they do not).
%
%   fig_sim.png              synthetic experiment: roll, pitch and errors vs truth
%   fig_bias.png             gyro bias estimates (synthetic, and exp. D with an artificial bias)
%   fig_real_C/D/E.png       roll and pitch per experiment: inclinometer (grey), onboard (black), KF (blue)
%   simulink_sim.png, simulink_replay.png   the two models (appendix, 300 dpi)

arguments
    opts.sweeps (1,1) logical = true
    opts.outdir (1,1) string = ""
    opts.reuse (1,1) logical = false   % reuse report/report_data.mat instead of re-running
end

here = fileparts(mfilename('fullpath'));
if opts.outdir == "", outdir = fullfile(here, 'report', 'overleaf', 'figs'); else, outdir = char(opts.outdir); end
if ~isfolder(outdir), mkdir(outdir); end
if ~isfolder(fullfile(here, 'report')), mkdir(fullfile(here, 'report')); end
addpath(here);

%% 1. Experiments
datafile = fullfile(here, 'report', 'report_data.mat');
if opts.reuse && isfile(datafile)
    load(datafile, 'R', 'S');
else
q = {'pacing', 0, 'scopes', false, 'quiet', true};
R = struct;
R.s0 = run_batista_kf_sim(q{:});                                         % default
R.s1 = run_batista_kf_sim(q{:}, 'accbias', [-0.033 -0.166 0]);          % accelerometer bias
R.s2 = run_batista_kf_sim(q{:}, 'noise', 0.01);                          % noise x 0.01
for e = 'CDE'
    R.(e) = run_batista_kf(e, q{:}, 'calibrate', 1.5);
end
R.Db = run_batista_kf('D', q{:}, 'calibrate', 1.5, 'bias', [3 -2 1]);    % artificial gyro bias

S = struct('facs', [1 10 100 1e3 1e4 1e5], 'bqs', [1e-8 1e-7 1e-6 1e-5 1e-4]);
S.f = nan(3, numel(S.facs));  S.q = nan(3, numel(S.bqs));                % rows: sim, D, E
if opts.sweeps
    % NaN where the fixed-step ode1 diverges (K = P/Th too large for the 5 ms step)
    for j = 1:numel(S.facs)
        S.f(1,j) = safe_rms(@() run_batista_kf_sim(q{:}, 'thfactor', S.facs(j)));
        S.f(2,j) = safe_rms(@() run_batista_kf('D', q{:}, 'calibrate', 1.5, 'thfactor', S.facs(j)));
        S.f(3,j) = safe_rms(@() run_batista_kf('E', q{:}, 'calibrate', 1.5, 'thfactor', S.facs(j)));
    end
    for j = 1:numel(S.bqs)
        S.q(1,j) = safe_rms(@() run_batista_kf_sim(q{:}, 'bq', S.bqs(j)));
        S.q(2,j) = safe_rms(@() run_batista_kf('D', q{:}, 'calibrate', 1.5, 'bq', S.bqs(j)));
        S.q(3,j) = safe_rms(@() run_batista_kf('E', q{:}, 'calibrate', 1.5, 'bq', S.bqs(j)));
    end
    fprintf('Sweep, RMS error [deg] (rows: synthetic, D, E; NaN = diverged):\n  factor %s\n', mat2str(S.facs));
    disp(S.f);  fprintf('  bq %s\n', mat2str(S.bqs));  disp(S.q);
end
save(fullfile(here, 'report', 'report_data.mat'), 'R', 'S', '-v7');
print_tables(R);

end

%% 2. Figures, in the style of kalman_filters.m (Section 4): default MATLAB look, 1000 px wide
% windows, grey = raw inclinometer, black = truth / onboard estimate, blue = Batista KF, grid on,
% legend 'best', labels in brackets, PNG at 150 dpi.
grey = [0.75 0.75 0.75];
datadir = fullfile(here, '..', 'data');
addpath(datadir);

% 2.1 Synthetic experiment: roll and pitch vs truth, and the errors
r = R.s0;  w = r.t >= 10 & r.t <= 25;
f = figure('Name', 'sim', 'Position', [50 50 1000 420]);
subplot(2,1,1);
plot(r.t(w), r.phi_true(w), 'k', 'LineWidth', 1.5); hold on;
plot(r.t(w), r.phi_hat(w), 'b');
plot(r.t(w), r.theta_true(w), 'k', 'LineWidth', 1.5);
plot(r.t(w), r.theta_hat(w), 'r');
xlim([10 25]); ylabel('\phi, \theta [deg]'); grid on;
legend('true state', 'KF \phi', 'KF \theta', 'Location', 'best');
title('Synthetic experiment, roll and pitch');
subplot(2,1,2);
plot(r.t, r.phi_hat - r.phi_true, 'b'); hold on;
plot(r.t, r.theta_hat - r.theta_true, 'r');
xlim([0 r.t(end)]); ylim([-0.3 0.3]); ylabel('error [deg]'); xlabel('t [s]'); grid on;
legend('\phi', '\theta', 'Location', 'best');
save_fig(f, outdir, 'fig_sim');

% 2.2 Gyro bias estimates: synthetic experiment and experiment D with an artificial bias
f = figure('Name', 'bias', 'Position', [50 50 1000 420]);
subplot(2,1,1);
btrue = [2 -3 1];  cs = 'brg';
for i = 1:3, plot(R.s0.t, btrue(i)*ones(size(R.s0.t)), 'k', 'LineWidth', 1.5); hold on; end
for i = 1:3, h(i) = plot(R.s0.t, R.s0.b_hat(:,i), cs(i)); end
xlim([0 R.s0.t(end)]); ylabel('gyro bias [deg/s]'); grid on;
legend(h, 'b_x', 'b_y', 'b_z', 'Location', 'best');
title('Synthetic experiment, bias estimate (black: true bias)');
subplot(2,1,2);
bart = [3 -2 1];
for i = 1:3, plot(R.Db.t, bart(i)*ones(size(R.Db.t)), 'k', 'LineWidth', 1.5); hold on; end
for i = 1:3, h(i) = plot(R.Db.t, R.Db.b_hat(:,i), cs(i)); end
xlim([0 R.Db.t(end)]); ylabel('gyro bias [deg/s]'); xlabel('t [s]'); grid on;
legend(h, 'b_x', 'b_y', 'b_z', 'Location', 'best');
title('Experiment D with an artificial bias added to the gyros (black: artificial bias)');
save_fig(f, outdir, 'fig_bias');

% 2.3 Real data, one figure per experiment: roll and pitch
for e = 'CDE'
    r = R.(e);  m = raw_inclinometer(datadir, e, r.t);
    f = figure('Name', ['exp ' e], 'Position', [50 50 1000 420]);
    for row = 1:2
        subplot(2,1,row);
        if row == 1, meas = m.phi;   ref = r.phi_ref;   hat = r.phi_hat;   yl = '\phi [deg]'; ttl = 'roll';
        else,        meas = m.theta; ref = r.theta_ref; hat = r.theta_hat; yl = '\theta [deg]'; ttl = 'pitch'; end
        plot(r.t, meas, 'Color', grey); hold on;
        plot(r.t, ref, 'k', 'LineWidth', 1.5);
        plot(r.t, hat, 'b');
        xlim([8 r.t(end) - 2]); ylabel(yl); grid on;
        legend('measurement (inclinometer)', 'flight state (onboard)', 'Batista KF', 'Location', 'best');
        title(['Experiment ' e ', ' ttl]);
        if row == 2, xlabel('t [s]'); end
    end
    save_fig(f, outdir, ['fig_real_' e]);
end

%% 9. Simulink diagrams for the appendix (logging indicators hidden, model not saved)
for n = {'ARDroneSim_BatistaKF', 'ARDroneReplay_BatistaKF'}
    m = n{1};
    bdclose(m);  load_system(fullfile(here, [m '.slx']));
    for l = find_system(m, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line')'
        p = get_param(l, 'SrcPortHandle');
        if p > 0, set_param(p, 'DataLogging', 'off'); end
    end
    orthogonalise_lines(m);                         % only horizontal and vertical segments
    set_param(m, 'PaperPositionMode', 'auto');      % page = diagram extent
    fn = strrep(strrep(m, 'ARDroneSim_BatistaKF', 'simulink_sim'), 'ARDroneReplay_BatistaKF', 'simulink_replay');
    print(['-s' m], '-dpng', '-r300', fullfile(outdir, [fn '.png']));
    bdclose(m);                                % discard the in-memory change
end
fprintf('Figures written to %s\n', outdir);
end

%% ------------------------------------------------------------------------
function orthogonalise_lines(m)
% Replace every diagonal segment of the signal lines by a horizontal-vertical-horizontal path.
for l = find_system(m, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line')'
    P = get_param(l, 'Points');  Q = P(1,:);
    for k = 2:size(P,1)
        if P(k,1) ~= P(k-1,1) && P(k,2) ~= P(k-1,2)
            xm = round((P(k-1,1) + P(k,1))/2);
            Q = [Q; xm P(k-1,2); xm P(k,2)]; %#ok<AGROW>
        end
        Q = [Q; P(k,:)]; %#ok<AGROW>
    end
    if size(Q,1) ~= size(P,1), set_param(l, 'Points', Q); end
end
end

function save_fig(f, outdir, name)
exportgraphics(f, fullfile(outdir, [name '.png']), 'Resolution', 150);
close(f);
end

function m = raw_inclinometer(datadir, e, t)
% Raw roll and pitch (deg) from the accelerometers, on the time vector t of a replay log.
% Same formulas as kalman_filters.m / Section 3.4 (sensor convention az = -g at rest).
d = decode_navdata(fullfile(datadir, sprintf('dados_alinea_%s.mat', e)));
m.phi   = interp1(d.t, atan2d(-d.acc(:,2), -d.acc(:,3)), t, 'linear', 'extrap');
m.theta = interp1(d.t, asind(max(min(d.acc(:,1)/9.81, 1), -1)), t, 'linear', 'extrap');
end

function v = safe_rms(run)
% Combined RMS error [deg] of a run, or NaN if the simulation diverged.
try
    r = run();
    v = sqrt(mean(r.rms.^2));
catch
    v = NaN;
end
end

function print_tables(R)
% Numbers of the tables of the report, for checking against the text.
fprintf('\nSynthetic experiment (RMS phi/theta, mean error phi/theta, bias estimate [deg/s]):\n');
for n = {'s0', 's1', 's2'}
    r = R.(n{1});
    fprintf('  %s: RMS %.2f / %.2f | mean %+.2f / %+.2f | bias [%.2f %.2f %.2f]\n', n{1}, r.rms, r.offset, r.bias_end);
end
fprintf('Real data, accelerometer calibrated (RMS offset removed, offset; deg):\n');
for e = 'CDE'
    r = R.(e);
    fprintf('  %s: RMS %.2f / %.2f | offset %+.2f / %+.2f | bias end [%.2f %.2f %.2f]\n', e, r.rms, r.offset, r.b_hat(end,:));
end
datadir = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
fprintf('Raw inclinometer vs onboard (offset removed RMS / offset; deg), for comparison with the KF:\n');
for e = 'CDE'
    r = R.(e);  m = raw_inclinometer(datadir, e, r.t);  w = r.t > 10 & r.t < r.t(end) - 3;
    ep = m.phi(w) - r.phi_ref(w);  et = m.theta(w) - r.theta_ref(w);
    fprintf('  %s: RMS %.2f / %.2f | offset %+.2f / %+.2f | peak %.1f / %.1f\n', e, std(ep), std(et), mean(ep), mean(et), max(abs(ep)), max(abs(et)));
end
r = R.Db;
fprintf('  D, artificial bias [3 -2 1]: bias end [%.2f %.2f %.2f]\n\n', r.b_hat(end,:));
end
