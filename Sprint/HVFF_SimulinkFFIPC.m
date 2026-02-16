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
ManipulateTXTFile(inflowFile,'"Wind/ECD_VrPlus2mps"','"Wind/SteadyWind"');

% Turn off DOFs
elastoFile = 'IEA-15-240-RWT-Monopile_ElastoDyn.dat';
ManipulateTXTFile(elastoFile,'True                   TwFADOF1','False                   TwFADOF1');
ManipulateTXTFile(elastoFile,'True                   TwSSDOF1','False                   TwSSDOF1');

% select simulated lidar
LidarType       = 'CircularCW'; % [4BeamPulsed/CircularCW]

% simulation time
TMax                = 600; % [s]

% IPC parameters
IPC = [];
verticalGains = 0.06 : 0.01 : 0.16; %static gain for Vertical component
horizontalGains = 0.06 : 0.01 : 0.16; %static gain for Horizontal component

switch LidarType
    case '4BeamPulsed'
        % configuration from LDP_v1_4BeamPulsed.IN and FFP_v1_4BeamPulsed.IN
        LDP.NumberOfBeams       = 4;            % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 19.176;       % [deg]     Angle around centerline
        LDP.IndexGate           = 6;            % [-]       IndexGate
        LDP.FlagLPF             = 1;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.1232;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 5.5;          % [s]       Buffer time for filtered REWS signal
        IPC.FB.Kp = 5.3e-7;
        IPC.FB.Ti = 4;
    case 'CircularCW'
        % configuration from LDP_v1_CircularCW.IN and FFP_v1_CircularCW.IN
        LDP.NumberOfBeams       = 50;           % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 15;           % [deg]     Angle around centerline
        LDP.IndexGate           = 1;            % [-]       IndexGate
        LDP.FlagLPF             = 0;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.3268;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        % LDP.omega_cutoff        = 0.5;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 7.5;          % [s]       Buffer time for filtered REWS signal        
        % LDP.T_buffer            = 7;          % [s]       Buffer time for filtered REWS signal     
        % Individual pitch controller
        IPC.FB.Kp = 5.3e-7;
        IPC.FB.Ti = 4;
end

% define FAST input file
SimulationName      = ['IEA-15-240-RWT-Monopile_Simulink_',LidarType];

% get Rosco Parameters
FAST_InputFileName  = [SimulationName,'.fst'];
fast.FAST_InputFile = FAST_InputFileName;
fast.FAST_directory = cd;
P                   = ReadWrite_FAST(fast);
simu.dt             = P.FP.Val{contains(P.FP.Label,'DT')};
[R,F]               = load_ROSCO_params(P,simu);

verticalShear       = 0.0167; % [(m/s)/m]

% add FF Parameter from FFP_v1.IN
R.StaticWind        = [0   10.0000   11.0000   12.0000   13.0000   14.0000   15.0000   16.0000   17.0000   18.0000   19.0000   20.0000   21.0000   22.0000   23.0000   24.0000   25.0000   26.0000   27.0000   28.0000   29.0000   30.0000]; % Wind speed  values in static pitch curve [m/s]
R.StaticPitch       = [0         0    0.0552    0.1085    0.1451    0.1749    0.2011    0.2250    0.2473    0.2682    0.2882    0.3072    0.3255    0.3432    0.3603    0.3769    0.3930    0.4087    0.4240    0.4389    0.4535    0.4679]; % Pitch angle values in static pitch curve [rad]

%% Run FBFF with IPC
for i =1:length(horizontalGains)
    clear FAST_SFunc 
    clear OpenFAST_ROSCO_LDP_FFP_with_FFIPC
    R.FlagLAC           = 1; % Enable LAC
    IPC.FF.gV           = verticalGains(i);
    IPC.FF.gH           = horizontalGains(i);

    SimOutFBFF          = sim('OpenFAST_ROSCO_LDP_FFP_with_FFIPC.slx',[0,TMax]);
    movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FBFFIPC.outb'])    % store results
    
    % read in data
    FBFFIPC         = ReadFASTbinaryIntoStruct([SimulationName,'_FBFFIPC.outb']);

    % Vertical Moment in last 3 rotation
    M_V     = SimOutFBFF.logsout.get('M_V').Values.Data;
    T       = 60/FBFFIPC.RotSpeed(end); % [s] time for 1 rev
    T_s     = round(TMax - 3*T); % Starting second for last 3 rev
    idx       = find( FBFFIPC.Time == T_s ); % Starting index
    M_V_final(i) = mean(M_V(idx:end));

    % Horizontal Moment in last 3 rotation
    M_H     = SimOutFBFF.logsout.get('M_H').Values.Data;
    M_H_final(i) = mean(M_H(idx:end));

    figure(i)
    subplot(4,1,1);
    hold on; grid on; box on
    plot(FBFFIPC.Time,       FBFFIPC.Wind1VelX);
    plot(SimOutFBFF.logsout.get('REWS_b').Values);
    ylabel('[m/s]');
    legend('Wind1VelX','REWS_b','Interpreter','none','Location','best')
    
    subplot(4,1,2);
    hold on; grid on; box on
    plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
    ylabel({'BldPitch1'; '[deg]'});
    
    subplot(4,1,3);
    hold on; grid on; box on
    plot(FBFFIPC.Time,     FBFFIPC.RotSpeed);
    ylabel({'RotSpeed';'[rpm]'});
    
    subplot(4,1,4);
    hold on; grid on; box on
    plot(FBFFIPC.Time,     FBFFIPC.TwrBsMyt/1e3);
    ylabel({'TwrBsMyt';'[MNm]'});
    
    xlabel('time [s]')
    linkaxes(findobj(gcf, 'Type', 'Axes'),'x');
    xlim([0 600])
end

%% Plot 
figure
hold on; grid on; box on
plot(verticalGains, M_V_final,'-o',LineWidth=2);
xlabel('Vertical Gain');
ylabel('Vertical Moment [kNm]');
% ResizeAndSaveFigure(16,9,'HVFF_Vgain_Results.fig')

figure
hold on; grid on; box on
plot(horizontalGains, M_H_final,'-o',LineWidth=2);
xlabel('Horizontal Gain');
ylabel('Horizontal Moment [kNm]');
% ResizeAndSaveFigure(16,9,'HVFF_Hgain_Results.fig')

%%
% revert the inflow change
ManipulateTXTFile(inflowFile,'"Wind/SteadyWind"','"Wind/ECD_VrPlus2mps"');

% Turn back on DOFs
ManipulateTXTFile(elastoFile,'False                   TwFADOF1','True                   TwFADOF1');
ManipulateTXTFile(elastoFile,'False                   TwSSDOF1','True                   TwSSDOF1');

% % display results
% RotSpeed_0  = 7.56;     % [rpm]
% TwrBsMyt_0  = 158.3e3;  % [kNm]
% t_Start     = 0;        % [s]
% 
% Cost = (max(abs(FBFFIPC.RotSpeed(FBFFIPC.Time>=t_Start)-RotSpeed_0))) / RotSpeed_0 ...
%      + (max(abs(FBFFIPC.TwrBsMyt(FBFFIPC.Time>=t_Start)-TwrBsMyt_0))) / TwrBsMyt_0;
% 
% fprintf('Cost for Summer Games 2024 ("FF_IPC"):  %f \n',Cost);