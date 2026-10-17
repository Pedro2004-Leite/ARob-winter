% Question 4.3 - compare the pitch response of the model from 4.2 with the
% real response of the drone to the reference: 0 for t < 15 s,
% 0.2 for 15 <= t < 18 s, 0 for t >= 18 s.
%
% Model:  G(s) = K*(s + z) / (s^2 + 2*zeta*wn*s + wn^2)
%
% The model is driven by the ideal reference above, as in the handout.
% The real run (pitch4_3.mat) recorded +0.2 for 15 to 18 s and then -0.2 for
% 18 to 21 s (a doublet), so only its first step is compared: the real theta
% follows the recorded hover oscillation before the step (from tPlot0; the
% trim offset is removed, so it fluctuates around 0), during the step (15 to 18 s) and then until theta
% first crosses 0 rad (the drone is being pushed down by the -0.2 of the
% doublet, so this is not a free return to zero); after the crossing the
% curve is set to 0. The RMSE is computed over the step only.
%
% The .mat file must contain the recorded run as Structure With Time,
% either as theta_data / thetaref_data (raw workspace variables) or as
% S / Sref (saved by record_run_pitch).
% If matFile is empty, only the model is simulated.

clear; clc; close all;

% Data file
matFile = fullfile(fileparts(mfilename('fullpath')), '..', 'experiments_Pedro', 'real', 'pitch4_3.mat');   % '' = model only

% Model parameters (from 4.2)
K    = 0.9088;
z    = 21.19;         % rad/s (zero at s = -z, minimum phase)
wn   = 6.822;         % rad/s
zeta = 0.4916;

% Time window of the plot (s). The real theta starts at tPlot0: before it comes
% the transient of switching the commands on, so only the hover oscillation
% before the step is kept.
tPlot0 = 12.5;
tPlot1 = 18.3;

% Trim offset
removeOffset = true;  % subtract the mean theta before the step (trim)
tOffWin = 2;          % s before the step used to estimate the trim (stops 0.2 s before it)

% Time grid and ideal reference
Ts = 0.03;
tg = (0:Ts:30)';
u  = 0.2*(tg >= 15 & tg < 18);

% Model simulation
s  = tf('s');
G  = K*(s + z) / (s^2 + 2*zeta*wn*s + wn^2);
ym = lsim(G, u, tg - tg(1));

hasReal = ~isempty(matFile);
if hasReal
    % Load the recorded run
    d = load(matFile);
    if isfield(d, 'theta_data') && isfield(d, 'thetaref_data')
        Sy = d.theta_data;  Su = d.thetaref_data;
    elseif isfield(d, 'S') && isfield(d, 'Sref')
        Sy = d.S;           Su = d.Sref;
    else
        error('%s must contain theta_data/thetaref_data or S/Sref.', matFile);
    end
    t    = Sy.time(:);
    y    = squeeze(Sy.signals.values);  y = y(:);
    uref = squeeze(Su.signals.values);  uref = uref(:);

    % First step of the recorded reference: it starts when the reference
    % leaves 0 and ends at the last sample before it changes value again
    i0   = find(abs(uref) > 0.5*max(abs(uref)), 1, 'first');
    i1   = find(abs(uref - uref(i0)) > 1e-6 & (1:numel(uref))' > i0, 1, 'first') - 1;
    tOn  = t(i0);
    if removeOffset
        y = y - mean(y(t >= tOn - tOffWin & t < tOn - 0.2));
    end

    % Put the recorded step on the time axis of the ideal reference
    % (t = 15 s at the start of the step). The recorded data are kept until
    % theta first crosses 0 rad after the step (linear interpolation between
    % the two samples around the crossing), then 0.
    tr   = t - tOn + 15;
    tEnd = tr(i1);
    i2   = i1 + find(y(i1+1:end) <= 0, 1);
    tCross = tr(i2-1) + y(i2-1) / (y(i2-1) - y(i2)) * (tr(i2) - tr(i2-1));
    yr   = nan(size(tg));
    yr(tg > tCross) = 0;
    inData = tg >= tPlot0 & tg <= tCross;
    yr(inData) = interp1(tr, y, tg(inData));
    inStep = tg >= 15 & tg <= tEnd;
end

% Plot (and RMSE between model and real, if there is real data). Same size and
% font sizes as the Bode figure of fit_pitch_bode.m.
fig = figure('Name', 'Pitch response to the reference of 4.3', 'Color', 'w', ...
    'Units', 'inches', 'Position', [0.5 0.5 12 7.5]);
ax = axes(fig);
if hasReal
    e = rmse(yr(inStep), ym(inStep));
    fprintf('RMSE (model vs real) = %.4f rad\n', e);
    plot(ax, tg, u, tg, yr, tg, ym, 'LineWidth', 1.5);
    legend(ax, 'Reference \theta_{ref}', 'Real \theta', 'Model \theta', 'Location', 'northwest', 'FontSize', 16);
    title(ax, 'Pitch Step Response: Identified Model vs Real Data', 'FontSize', 18);
else
    plot(ax, tg, u, tg, ym, 'LineWidth', 1.5);
    legend(ax, 'Reference \theta_{ref}', 'Model \theta', 'Location', 'northwest', 'FontSize', 16);
    title(ax, 'Pitch Step Response: Identified Model', 'FontSize', 18);
end
grid(ax, 'on'); ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on';
ax.FontSize = 16;
xlim(ax, [tPlot0 tPlot1]);
ylim(ax, [-0.05 0.25]);
xlabel(ax, 'Time t [s]', 'FontSize', 18);
ylabel(ax, 'Pitch \theta [rad]', 'FontSize', 18);

% Save the figure for the report in Lab1/report (only with the real data, so a
% model-only run does not overwrite it):
%   fig_pitch_step.png  raster, 600 dpi
%   fig_pitch_step.pdf  vector (no loss of quality at any zoom)
% Padding adds a margin around the axes, so labels and title are not cropped.
if hasReal
    outDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', '..', 'report');
    if ~isfolder(outDir)
        mkdir(outDir);
    end
    outPng = fullfile(outDir, 'fig_pitch_step.png');
    outPdf = fullfile(outDir, 'fig_pitch_step.pdf');
    try
        exportgraphics(fig, outPng, 'Resolution', 600, 'Padding', 30, 'BackgroundColor', 'white');
        exportgraphics(fig, outPdf, 'ContentType', 'vector', 'Padding', 30, 'BackgroundColor', 'white');
    catch
        % MATLAB without the Padding option: add the margin by hand.
        ax.Units = 'normalized'; ax.OuterPosition = [0.02 0.02 0.96 0.96];
        exportgraphics(fig, outPng, 'Resolution', 600, 'BackgroundColor', 'white');
        exportgraphics(fig, outPdf, 'ContentType', 'vector', 'BackgroundColor', 'white');
    end
    fprintf('Figures saved: %s and %s\n', outPng, outPdf);
end
