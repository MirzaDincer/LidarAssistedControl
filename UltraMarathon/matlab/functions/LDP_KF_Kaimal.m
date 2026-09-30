function REWS_b = LDP_KF_Kaimal(isValid, beamID, lineOfSightWindSpeed, DT, LDP)
% LDP_KF_Kaimal  Kalman-filter LDP: Taylor advection + Kaimal shaping filter.
%
% Called once per time step (same style as LDP_v3).
%   isValid, lineOfSightWindSpeed : 1 x M vectors (all range gates, this step)
%   beamID                        : scalar
%
% STATE  x = [ u'_1 ... u'_M  (wind fluctuation at the M gate planes)
%              x_turb(1:2)    (2nd-order Kaimal shaping-filter states) ]
% Mean wind Ubar is tracked separately (slow low-pass of the lidar data),
% so the states are fluctuations around Ubar.
%
% REQUIRED LDP FIELDS
%   AngleToCenterline [deg]  GateDistances [m] (1xM)   U0 [m/s]  Iref [-]
%   L_u [m]  q_per_m  R [(m/s)^2]  alphaU [-]

persistent x P Ubar PrevBeamID Ad Cc Qturb

d0 = 0;                                   % <<< CHANGED: extra plane at 0 m
d  = [d0; LDP.GateDistances(:)];          % <<< CHANGED: plane distances (M+1)
Mp = numel(d);                            % <<< CHANGED: number of planes (M+1)
dd = diff(d);                             % plane spacing, Mp-1 x 1
nx = Mp + 2;                              % planes + 2 Kaimal states
Hc = cosd(LDP.AngleToCenterline);         % LOS projection factor


%% ================= 0 - SET INITIALS (runs once) =========================
if isempty(x)
    % --- Kaimal target spectrum (IEC), evaluated at the design speed U0
    U0      = LDP.U0;
    sigma_u = LDP.Iref*(0.75*U0 + 5.6);                 % IEC turbulence std
    f       = logspace(-3, log10(2), 60)';  w = 2*pi*f; % fit range [Hz]
    S_kai   = 4*sigma_u^2*LDP.L_u/U0 ./ (1 + 6*f*LDP.L_u/U0).^(5/3);

    % --- Fit H(s) = 1/(s^2 + a1*s + a0) to the Kaimal SHAPE (log-spectrum).
    %     Crude 2-parameter fit; -5/3 slope cannot be matched exactly by 2nd order.
    shape = @(p) 1 ./ abs(exp(p(2)) - w.^2 + 1j*exp(p(1))*w).^2;
    cost  = @(p) var(log(S_kai) - log(shape(p)));       % offset-free mismatch
    w0    = 2*pi*0.05;                                  % start: double pole at 0.05 Hz
    p     = fminsearch(cost, [log(2*w0), log(w0^2)]);
    a1 = exp(p(1));  a0 = exp(p(2));

    % --- Continuous shaping filter (controllable canonical form)
    Ac = [0 1; -a0 -a1];   Bc = [0; 1];   Cc = [1 0];

    % --- Stationary covariance for unit white noise: Ac*Pc + Pc*Ac' + Bc*Bc' = 0
    Pc = reshape(-(kron(eye(2),Ac) + kron(Ac,eye(2))) \ reshape(Bc*Bc',[],1), 2, 2);
    Cc = Cc * sigma_u / sqrt(Cc*Pc*Cc');                % output variance = sigma_u^2

    % --- Discretize: Ad = expm(Ac*DT); Q_Kaimal = integral of e^(Ac t)Bc Bc'e^(Ac' t)
    %     which equals Pc - Ad*Pc*Ad' for the stationary process
    Ad    = expm(Ac*DT);
    Qturb = Pc - Ad*Pc*Ad';   Qturb = (Qturb + Qturb')/2;

    % --- Initial state and covariance
    x  = zeros(nx,1);
    P  = blkdiag(sigma_u^2*eye(Mp), Pc);                  % P0 as in the document
    Ubar       = U0;
    PrevBeamID = -1;
end

%% ================= 1 - PREDICT STATE / ERROR COVARIANCE (every step) ====
U = max(Ubar, 3);                        % advection speed, floored
a = U*DT ./ dd;                          % fraction of a gate spacing travelled per step

% Taylor advection block (upwind interpolation): plane i takes from plane i+1
Aadv = eye(Mp);
for i = 1:Mp-1
    Aadv(i,i)   = 1 - a(i);
    Aadv(i,i+1) = a(i);
end
Aadv(Mp,Mp) = 1 - a(Mp-1);                  % farthest plane: no upstream data

% New turbulence enters at the farthest plane through the Kaimal output
BC = zeros(Mp,2);
BC(Mp,:) = a(Mp-1)*Cc;

A = [Aadv,          BC;
     zeros(2,Mp),    Ad];

Q = blkdiag(diag([LDP.q_per_m*dd; LDP.q_per_m*dd(end)]), Qturb);

x = A*x;                                 % x_{k|k-1}
P = A*P*A' + Q;                          % P_{k|k-1}

%% ================= UPDATE: only on a NEW beam sample ====================
if beamID ~= PrevBeamID
    PrevBeamID = beamID;
    idx = find(isValid(:));              % invalid gates are simply skipped (R -> inf)

    if ~isempty(idx)
        nz = numel(idx);
        z  = lineOfSightWindSpeed(:);  z = z(idx);      % measurement vector
        H  = zeros(nz, nx);                              % observation matrix
        for j = 1:nz
           H(j, idx(j)+1) = Hc;                          % Kaimal columns stay zero
        end
        R = LDP.R * eye(nz);                             % measurement noise
        y_tilde = z - Hc*Ubar - H*x;                     % innovation (on fluctuations)

        %% ============= 2 - COMPUTE KALMAN GAIN ===========================
        S = H*P*H' + R;
        K = (P*H') / S;

        %% ============= 3 - COMPUTE THE ESTIMATE ==========================
        x = x + K*y_tilde;

        %% ============= 4 - COMPUTE THE ERROR COVARIANCE ==================
        P = (eye(nx) - K*H) * P;
        P = (P + P')/2;                                  % keep symmetric

        % slow causal update of the mean wind (lidar data only)
        Ubar = Ubar + LDP.alphaU * (mean(z)/Hc - Ubar);
    end
end

REWS_b = Ubar + x(1);
print
end

%% Functions

% Buffer function, taken from LDP_v3
function REWS_b = Buffer(REWS,DT,T_buffer)

% init REWS_f_Buffer
nBuffer = 400; % Size of REWS_f_buffer, max 20 seconds at 20 Hz  [-] 
persistent REWS_f_Buffer;
if isempty(REWS_f_Buffer)      
    REWS_f_Buffer = ones(nBuffer,1)*REWS;   
end 

% Update Buffer for estimated u component
REWS_f_Buffer    = [REWS;REWS_f_Buffer(1:nBuffer-1)];

% Index for entry at T_buffer, minimum 1, maximum nBuffer
Idx     = min( max(floor(T_buffer/DT),1) , nBuffer);
		
% Get buffered and filtered REWS from buffer
REWS_b  = REWS_f_Buffer(Idx);

end

