function res = run_tt_experiment(exper, ov, varargin)
%RUN_TT_EXPERIMENT  One-call run of the Lab 3 ARDroneTTSim model with logging + metrics.
%   res = run_tt_experiment(1)
%   res = run_tt_experiment(5, {'traj.los_V',0.2,'traj.los_Delta',0.5})
%   res = run_tt_experiment(2, {}, 'StopTime',150,'Plot',true,'Tskip',20)
% Works on a private copy of the devkit (repo untouched, no slprj/ slxc left in it).
% Callable from a function or loop: setup runs in the BASE workspace (like setupTTSim).
% Options: 'Devkit' (path, default: ../DevKit_Lab3/ARDroneSimulinkDevKit_V1.1 relative to this file),
%   'StopTime' (default 150), 'Tskip' (20), 'Plot' (false), 'Work' (dir),
%   'Save' (true: write results to OutDir), 'OutDir' (default ./results next to this file).
% Saved files are named after the experiment and the overridden parameters, e.g.
%   exp5__los_V-0.2__los_Delta-0.5__T150.{mat,csv,png}   (no overrides: exp1__default__T150)
% and one row per run is appended to OutDir/summary.csv (parameters + metrics).
p = inputParser;
here = fileparts(mfilename('fullpath'));
p.addParameter('Devkit',fullfile(here,'..','DevKit_Lab3','ARDroneSimulinkDevKit_V1.1'));
p.addParameter('Save',true); p.addParameter('OutDir',fullfile(here,'results'));
p.addParameter('StopTime',150); p.addParameter('Tskip',20); p.addParameter('Plot',false);
p.addParameter('Work',''); p.parse(varargin{:}); o = p.Results;
if nargin<2, ov = {}; end
if isstruct(ov), f=fieldnames(ov); c={}; for i=1:numel(f), c=[c {strrep(f{i},'__','.') ov.(f{i})}]; end, ov=c; end

persistent work
if isempty(work) || ~isfolder(work) || ~isempty(o.Work)
    if isempty(o.Work), work = tempname; else, work = o.Work; end
    if isfolder(work), rmdir(work,'s'); end
    copyfile(o.Devkit, work);
    for d = {'simulation/slprj','simulation/ARDroneTTSim.slxc','simulation/ARDroneTTSim.slx.autosave'}
        if exist(fullfile(work,d{1}),'file'), try, rmdir(fullfile(work,d{1}),'s'); catch, delete(fullfile(work,d{1})); end, end
    end
    % setupTTSim without 'clear all' / model load (so no workspace wipe from inside a function)
    s = fileread(fullfile(work,'simulation','setupTTSim.m'));
    s = regexprep(s,'^\s*(bdclose all;|clear all;|clc)\s*$','','lineanchors');
    s = regexprep(s,'^\s*ARDroneTTSim\s*;\s*$','','lineanchors');
    fid=fopen(fullfile(work,'simulation','setupTTSim_nc.m'),'w'); fwrite(fid,s); fclose(fid);
    bdclose all
    evalin('base',sprintf('cd(''%s''); setupTTSim_nc;',fullfile(work,'simulation')));
    evalin('base','simDT = simDT;');
end
cfgf = fullfile(work,'functions','tt_config.m');
if ~isfile([cfgf '.orig']), copyfile(cfgf,[cfgf '.orig']); end
txt = fileread([cfgf '.orig']);
txt = regexprep(txt,'EXPERIMENT = \d+;',sprintf('EXPERIMENT = %d;',exper));
extra = '';
for i=1:2:numel(ov), extra = [extra sprintf('%s = %s;\n',ov{i},mat2str(ov{i+1}))]; end %#ok<AGROW>
txt = strrep(txt,'cfg.experiment = EXPERIMENT;',[extra 'cfg.experiment = EXPERIMENT;']);
fid=fopen(cfgf,'w'); fwrite(fid,txt); fclose(fid);
rehash; clear functions   %#ok<CLFUNC>  % MATLAB Function blocks re-read tt_config

m = 'ARDroneTTSim';
if ~bdIsLoaded(m), load_system(fullfile(work,'simulation',m)); end
set_param(m,'StopTime',num2str(o.StopTime),'SignalLogging','on','SignalLoggingName','logsout', ...
    'SaveOutput','off','ReturnWorkspaceOutputs','on');
blks = find_system(m,'SearchDepth',1,'Type','block');
nm = get_param(blks,'Name');
pe = blks{startsWith(nm,'Position estimation')};
tr = blks{strcmp(nm,'MATLAB Function1')};
oc = blks{strcmp(nm,'Outer-loop Controller')};
setlog(pe,1,'est'); setlog(tr,1,'des'); for k=1:4, setlog(oc,k,sprintf('u%d',k)); end
out = sim(m);
ls = out.logsout;
g = @(n) ls.get(n).Values;
est = g('est'); fn = fieldnames(est);          % the estimator output is a bus of 9 timeseries
t = est.(fn{1}).Time; S = zeros(numel(t),9);
for j=1:9, S(:,j) = squeeze(est.(fn{j}).Data); end
des = g('des'); D = squeeze(des.Data); if size(D,1)==3, D=D'; end
D = interp1(des.Time,D,t,'previous','extrap');
U = [g('u1').Data(:) g('u2').Data(:) g('u3').Data(:) g('u4').Data(:)];
U = interp1(g('u1').Time,U,t,'previous','extrap');
res.t = t; res.exper = exper; res.overrides = ov;
res.sig = table(t,S(:,1),S(:,2),S(:,3),S(:,4),S(:,5),S(:,6),S(:,7),S(:,8),S(:,9),D(:,1),D(:,2),D(:,3),U(:,1),U(:,2),U(:,3),U(:,4), ...
  'VariableNames',{'t','phi','theta','psi','u','v','hdot','x','y','h','xd','yd','zd','w_r','dpsi_r','theta_r','phi_r'});
k = t>o.Tskip;
e = hypot(S(:,7)-D(:,1),S(:,8)-D(:,2)); eh = S(:,9)+D(:,3);   % zd is NED (h = -zd)
res.metrics = struct('exper',exper,'Tskip',o.Tskip,'horiz_mean',mean(e(k)),'horiz_max',max(e(k)), ...
  'height_max',max(abs(eh(k))),'max_roll',max(abs(S(:,1))),'max_pitch',max(abs(S(:,2))), ...
  'yaw_range',[min(S(k,3)) max(S(k,3))],'sat_theta',max(abs(U(:,3))),'sat_phi',max(abs(U(:,4))), ...
  't_conv',tconv(t,e,0.05));
res.e_h = e;
if o.Plot || o.Save
    fg = figure('Visible','off'); tl = tiledlayout(fg,2,2);
    nexttile; plot(S(:,7),S(:,8),D(:,1),D(:,2),'--'); axis equal; grid on; xlabel x; ylabel y; legend est des
    nexttile; plot(t,e); grid on; xlabel t; ylabel('horiz err [m]');
    nexttile; plot(t,S(:,9),t,-D(:,3),'--'); grid on; ylabel h; xlabel t
    nexttile; plot(t,S(:,1:3)); grid on; legend phi theta psi; xlabel t
    title(tl,sprintf('exp %d  %s',exper,strrep(runname(exper,ov,o.StopTime),'_','\_'))); res.fig = fg;
end
res.name = runname(exper,ov,o.StopTime);
if o.Save
    if ~isfolder(o.OutDir), mkdir(o.OutDir); end
    base = fullfile(o.OutDir,res.name);
    r = res; r = rmfield(r,'fig'); save([base '.mat'],'-struct','r');
    writetable(res.sig,[base '.csv']);
    exportgraphics(res.fig,[base '.png'],'Resolution',150);
    res.files = {[base '.mat'],[base '.csv'],[base '.png']};
    sumf = fullfile(o.OutDir,'summary.csv');
    mt = res.metrics;
    row = table({res.name},exper,{strjoin(paramstr(ov),' ')},o.StopTime,mt.horiz_mean,mt.horiz_max,mt.height_max, ...
        mt.max_roll,mt.max_pitch,mt.sat_theta,mt.sat_phi,mt.t_conv,'VariableNames', ...
        {'name','experiment','overrides','StopTime','horiz_mean','horiz_max','height_max','max_roll','max_pitch','sat_theta','sat_phi','t_conv'});
    if isfile(sumf), writetable(row,sumf,'WriteMode','append','WriteVariableNames',false); else, writetable(row,sumf); end
end
if ~o.Plot && isfield(res,'fig'), close(res.fig); res = rmfield(res,'fig'); end
end
function n = runname(exper,ov,T)
parts = paramstr(ov); if isempty(parts), parts = {'default'}; end
n = sprintf('exp%d__%s__T%g',exper,strjoin(parts,'__'),T);
n = regexprep(n,'[^A-Za-z0-9_.\-]','');
end
function c = paramstr(ov)
c = cell(1,numel(ov)/2);
for i=1:2:numel(ov)
    nmx = regexprep(ov{i},'^(traj|ctrl)\.',''); c{(i+1)/2} = sprintf('%s-%s',nmx,num2str(ov{i+1}));
end
if isempty(ov), c = {}; end
end
function setlog(blk,port,name)
ph = get_param(blk,'PortHandles'); set_param(ph.Outport(port),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',name);
end
function tc = tconv(t,e,tol)
k = find(e>tol,1,'last'); if isempty(k), tc=0; elseif k==numel(t), tc=NaN; else, tc=t(k+1); end
end
