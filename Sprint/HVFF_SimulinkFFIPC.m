% Sprint: DLC 1.4 for IEA 15 MW monopile with and without LAC. 
% Purpose:
% Finding optimal gains for HV FF controller 
% Aim is to achieve zero Vertical/Horizontal moments
% 12.5 m/s Steady wind


%% Setup
clearvars;close all;clc;
addpath(genpath('..\WetiMatlabFunctions'))
addpath(genpath('..\NrelMatlabFunctions'))

% change to steady wind
inflowFile = 'IEA-15-240-RWT_InflowFile.dat';
wind_cases = ["Wind/SteadyWind_0p012","Wind/SteadyWind_0p016", "Wind/SteadyWind_0p020", "Wind/SteadyWind_0p024"];
% wind_cases = ["Wind/SteadyWind_0p024"];
shear_tags = "sh" + ["0p012", "0p016", "0p020", "0p024"];
ManipulateTXTFile(inflowFile,'"Wind/ECD_VrPlus2mps"',wind_cases(1));


% Turn off DOFs
elastoFile = 'IEA-15-240-RWT-Monopile_ElastoDyn.dat';
ManipulateTXTFile(elastoFile,'True                   TwFADOF1','False                   TwFADOF1');
ManipulateTXTFile(elastoFile,'True                   TwSSDOF1','False                   TwSSDOF1');

ManipulateTXTFile(elastoFile,'-6.0                   ShftTilt','0.0                   ShftTilt');

% Changing cone angles manually to zero. 

% select simulated lidar
LidarType       = 'CircularCW'; % [4BeamPulsed/CircularCW]

% simulation time
TMax                = 600; % [s]

% IPC parameters
IPC = [];
% verticalGains = 1 : 0.1 : 1.5; %static gain for Vertical component
verticalGains = 1.4;
gain_tags    = "g" + strrep(string(verticalGains), '.', 'p');  % "g0p0", "g0p5", "g1p0", etc.



switch LidarType
    case '4BeamPulsed'
        % configuration from LDP_v1_4BeamPulsed.IN and FFP_v1_4BeamPulsed.IN
        LDP.NumberOfBeams       = 4;            % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 19.176;       % [deg]     Angle around centerline
        LDP.IndexGate           = 6;            % [-]       IndexGate
        LDP.FlagLPF             = 1;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.1232;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 5.5;          % [s]       Buffer time for filtered REWS signal
        [Y, Z]                  = calculateposlidbeams('LidarFile_4BeamPulsed.dat' , LDP.IndexGate, LDP.NumberOfBeams);
        LDP.Ycoord              = Y;
        LDP.Zcoord              = Z;
        IPC.FB.Kp               = 5.3e-7;
        IPC.FB.Ti               = 4;
        LidarFile               = 'LidarFile_4BeamPulsed.dat';
    case 'CircularCW'
        % configuration from LDP_v1_CircularCW.IN and FFP_v1_CircularCW.IN
        LDP.NumberOfBeams       = 50;           % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 15;           % [deg]     Angle around centerline
        LDP.IndexGate           = 1;            % [-]       IndexGate
        LDP.FlagLPF             = 0;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.3268;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 7.5;          % [s]       Buffer time for filtered REWS signal        
        % Individual pitch controller
        IPC.FB.Kp = 1e-7;
        IPC.FB.Ti = 4;
        LidarFile               = 'LidarFile_CircularCW.dat';
        [Y, Z]                  = calculateposlidbeams('LidarFile_CircularCW.dat' , LDP.IndexGate, LDP.NumberOfBeams);
        LDP.Ycoord              = Y;
        LDP.Zcoord              = Z;
end

% change Lidar file
ManipulateTXTFile(LidarFile,'2       WeightingType','0       WeightingType'); % disable lidar volume for a point measurement
ManipulateTXTFile(LidarFile,'True        NearestInterpFlag','False        NearestInterpFlag'); % change the grid interpolation to linear


% define FAST input file
SimulationName      = ['IEA-15-240-RWT-Monopile_Simulink_',LidarType];

% get Rosco Parameters
FAST_InputFileName  = [SimulationName,'.fst'];
fast.FAST_InputFile = FAST_InputFileName;
fast.FAST_directory = cd;
P                   = ReadWrite_FAST(fast);
simu.dt             = P.FP.Val{contains(P.FP.Label,'DT')};
[R,F]               = load_ROSCO_params(P,simu);

% phase offset
deltat = 0.425;                          % s — use your measured value
LDP.deltaphi = -R.PC_RefSpd * deltat;   % rad — at rated speed

% add FF Parameter from FFP_v1.IN
R.StaticWind        = [0   10.0000   11.0000   12.0000   13.0000   14.0000   15.0000   16.0000   17.0000   18.0000   19.0000   20.0000   21.0000   22.0000   23.0000   24.0000   25.0000   26.0000   27.0000   28.0000   29.0000   30.0000]; % Wind speed  values in static pitch curve [m/s]
R.StaticPitch       = [0         0    0.0552    0.1085    0.1451    0.1749    0.2011    0.2250    0.2473    0.2682    0.2882    0.3072    0.3255    0.3432    0.3603    0.3769    0.3930    0.4087    0.4240    0.4389    0.4535    0.4679]; % Pitch angle values in static pitch curve [rad]

%% Run FBFF with IPC 
for i =1:length(wind_cases) % Run for nGains x 4 shears
    for j =1:length(verticalGains)
    clear FAST_SFunc 
    clear OpenFAST_ROSCO_LDP_FFP_with_FFIPC
    R.FlagLAC           = 1; % Enable LAC
    IPC.FF.gV           = verticalGains(j) ; % verticalGains(j)
    IPC.FF.gH           = 0;

    ManipulateTXTFile(inflowFile,'"Wind/ECD_VrPlus2mps"',wind_cases(i));

    SimOutFBFF          = sim('OpenFAST_ROSCO_LDP_FFP_with_FFIPC.slx',[0,TMax]);
    % SimOutFBFF          = sim('OpenFAST_ROSCO_LDP_FFP.slx',[0,TMax]);
    movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FBFFIPC.outb'])    % store results
    
    % read in data
    FBFFIPC         = ReadFASTbinaryIntoStruct([SimulationName,'_FBFFIPC.outb']);

 % Vertical Moment in last 3 rotations
    M_V     = SimOutFBFF.logsout.get('M_V').Values.Data;
    M_H     = SimOutFBFF.logsout.get('M_H').Values.Data;
    FBFFIPC.M_V = SimOutFBFF.logsout.get('M_V').Values.Data;
    T       = 60 / FBFFIPC.RotSpeed(end);          % [s] time for 1 rev
    T_s     = TMax - 3*T;                           % Starting second for last 3 rev
    idx     = find(FBFFIPC.Time >= T_s, 1, 'first'); % Robust floating point search
    
    M_V_final(i,j)        = mean(M_V(idx:end));
    M_H_final(i,j)        =mean(M_H(idx:end));
    moop_final            = FBFFIPC.RootMyb1(idx:end);
    moop_final_mean(i,j)  = mean(moop_final);
    moop_final_amp(i,j)   = max(moop_final) - min(moop_final);
    pitch_final(i,j)      = FBFFIPC.BldPitch1(end);
    
    results.(shear_tags(i)).(gain_tags(j)) = FBFFIPC;


%% plot results

    % wind speed + blade pitch + rotational speed + tower base moment 
    % figure
    % subplot(4,1,1);
    % hold on; grid on; box on
    % plot(FBFFIPC.Time,       FBFFIPC.Wind1VelX);
    % plot(SimOutFBFF.logsout.get('REWS_b').Values);
    % ylabel('[m/s]');
    % legend('Wind1VelX','REWS_b','Interpreter','none','Location','best')
    % 
    % subplot(4,1,2);
    % hold on; grid on; box on
    % plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
    % ylabel({'BldPitch1'; '[deg]'});
    % 
    % subplot(4,1,3);
    % hold on; grid on; box on
    % plot(FBFFIPC.Time,     FBFFIPC.RotSpeed);
    % ylabel({'RotSpeed';'[rpm]'});
    % 
    % subplot(4,1,4);
    % hold on; grid on; box on
    % plot(FBFFIPC.Time,     FBFFIPC.TwrBsMyt/1e3);
    % 
    % ylabel({'TwrBsMyt';'[MNm]'});
    % xlabel('time [s]')
    % linkaxes(findobj(gcf, 'Type', 'Axes'),'x');
    % xlim([500 600])

    % BldPitch1 + RootMyb1 + M_V
    figure
    subplot(3,1,1)
    hold on; grid on; box on
    plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
    ylabel({'BldPitch1'; '[deg]'});
    ylim([4 12])
    xlim([500 600])

    subplot(3,1,2)
    hold on; grid on; box on
    plot(FBFFIPC.Time, FBFFIPC.RootMyb1);
    ylabel({'Oop1 Moment'; '[kNm]'});
    xlim([500 600])

    subplot(3,1,3)
    hold on; grid on; box on
    plot(FBFFIPC.Time, SimOutFBFF.logsout.get('M_V').Values.Data);
    ylabel({'Vertical Moment'; '[kNm]'});
    xlim([500 600])

    ManipulateTXTFile(inflowFile,wind_cases(i),'"Wind/ECD_VrPlus2mps"');

    end
end

%% Plot vertical gains - vertical moments
% figure
% hold on; grid on; box on
% plot(verticalGains, M_V_final,'-o',LineWidth=2);
% xlabel('Vertical Gain');
% ylabel('Vertical Moment [kNm]');

%% Plot vertical gains - oop moment amplitude
% figure
% hold on; grid on; box on
% plot(verticalGains, moop_final_amp,'-o',LineWidth=2);
% xlabel('Vertical Gain');
% ylabel('Oop1 Moment amplitude [kNm]');


%% azimuth and pitch

% figure
% subplot(2,1,1)
% hold on; grid on; box on
% plot(SimOutFBFF.logsout.get('Azimuth').Values);
% subplot(2,1,2)
% hold on; grid on; box on
% plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
% ylabel({'BldPitch1'; '[deg]'});

%% plot shear

% figure;
% hold on; grid on; box on
% plot(SimOutFBFF.logsout.get('deltaV').Values);
% xlabel('time [s]')
% ylabel('Shear [(m/s)/m]')
% legend('Vertical Shear','Location','northwest');
% ylim([0.012 0.026])

% figure;
% hold on; grid on; box on
% plot(SimOutFBFF.logsout.get('v_los').Values);
% xlabel('time [s]')
% ylabel('Shear [(m/s)/m]')
% legend('v_los','Location','northwest');

%% without FFIPC check
% subplot(3,1,1)
% hold on; grid on; box on
% plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
% ylabel({'BldPitch1'; '[deg]'});
% subplot(3,1,2)
% hold on; grid on; box on
% plot(FBFFIPC.Time, FBFFIPC.RootMyb1);
% ylabel({'Oop1 Moment'; '[kNm]'});
% subplot(3,1,3)
% hold on; grid on; box on
% plot(FBFFIPC.Time, SimOutFBFF.logsout.get('M_V').Values.Data);
% ylabel({'Vertical Moment'; '[kNm]'});


%%
% Turn back on DOFs
ManipulateTXTFile(elastoFile,'False                   TwFADOF1','True                   TwFADOF1');
ManipulateTXTFile(elastoFile,'False                   TwSSDOF1','True                   TwSSDOF1');
ManipulateTXTFile(elastoFile,'0.0                   ShftTilt','-6.0                   ShftTilt');

ManipulateTXTFile(LidarFile,'0       WeightingType','2       WeightingType'); % disable lidar volume for a point measurement
ManipulateTXTFile(LidarFile,'False        NearestInterpFlag','True        NearestInterpFlag'); % change the grid interpolation to linear
