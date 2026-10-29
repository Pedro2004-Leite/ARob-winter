function m = decode_navdata(file)
% DECODE_NAVDATA  Transform the raw navdata bytes of a Lab 2 experiment into
% the actual instrument measurements, in SI units.
%
%   m = decode_navdata('dados_alinea_A.mat')
%
% FILE is a data file of this folder (name or full path). The file must
% contain the Simulink variable "navdata" (N x 2186 uint8 packets, 200 Hz).
%
% Output struct M (N = number of valid packets; startup packets that are
% still all-zero are dropped from every field):
%   m.t        N x 1  time [s]
%   m.acc      N x 3  accelerometer [ax ay az], m/s^2  (phys_measures option)
%   m.gyro     N x 3  rate gyro [gx gy gz],     rad/s  (phys_measures option)
%   m.alt_us   N x 1  ultrasound altitude,      m      (altitude option, raw)
%
% Sign convention is the sensor's own: at rest az ~ -g (not corrected).
%
% How to access each field (m = decode_navdata('dados_alinea_C.mat')):
%   m.t                 time vector                       [s]
%   m.acc               all accelerometers, N x 3         [m/s^2]
%   m.acc(:,1)          ax  (x axis)
%   m.acc(:,2)          ay  (y axis)
%   m.acc(:,3)          az  (z axis)
%   m.gyro              all rate gyros, N x 3             [rad/s]
%   m.gyro(:,1)         gx  (roll rate,  x axis)
%   m.gyro(:,2)         gy  (pitch rate, y axis)
%   m.gyro(:,3)         gz  (yaw rate,   z axis)
%   m.alt_us            ultrasound altitude, N x 1        [m]
%
% Examples:
%   plot(m.t, m.acc(:,3))                 % az over time
%   X = [m.acc m.gyro];                   % N x 6 stacked [ax ay az gx gy gz]
%   idx = m.t >= 10 & m.t <= 37;          % select a time window
%   mu = mean(X(idx,:));  C = cov(X(idx,:));

    here = fileparts(mfilename('fullpath'));
    if ~isfile(file), file = fullfile(here, file); end
    S = load(file, 'navdata');
    t = S.navdata.time(:);
    raw = S.navdata.signals.values;                 % N x 2186 uint8

    N = size(raw, 1);
    acc = nan(N,3);  gyro = nan(N,3);  alt = nan(N,1);

    for k = 1:N
        row = raw(k,:);
        pos = 13;                                   % options start at byte 12 (1-based 13)
        while pos + 3 <= numel(row)
            tag = double(typecast(row(pos:pos+1),   'uint16'));
            len = double(typecast(row(pos+2:pos+3), 'uint16'));  % includes 4-byte header
            if len < 4, break; end
            b = pos + 4;                            % option body start
            if tag == 3                             % phys_measures
                acc(k,:)  = double(typecast(row(b+6  : b+17), 'single'));  % mg
                gyro(k,:) = double(typecast(row(b+18 : b+29), 'single'));  % deg/s
            elseif tag == 10                        % altitude
                alt(k) = double(typecast(row(b+12 : b+15), 'int32'));      % mm (ultrasound)
            end
            pos = pos + len;
        end
    end

    ok = ~any(isnan([acc gyro alt]), 2);            % drop empty startup packets

    m.t      = t(ok);
    m.acc    = acc(ok,:)  * 9.81/1000;              % mg    -> m/s^2
    m.gyro   = gyro(ok,:) * pi/180;                 % deg/s -> rad/s
    m.alt_us = alt(ok)    / 1000;                   % mm    -> m
end
