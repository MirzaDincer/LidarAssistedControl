% Sprint: DLC 1.4 for IEA 15 MW monopile with and without LAC. 
% Purpose:
% We want to learn how to simulate a DLC 1.4 with an "Extreme coherent gust 
% with direction change (ECD)" with lidar-assisted control (LAC) and how
% LAC can reduce the ultimate tower loads. 
% Here, only the rotor motion and tower motion (GenDOF, TwFADOF1, TwSSDOF1) 
% are enabled for simplicity.
% Result (slightly different to pure matlab version RunExample.m):       
% Cost for Summer Games 2025 ("30 s sprint"):  0.722838 (4BeamPulsed)
% Cost for Summer Games 2025 ("30 s sprint"):  1.217274 (CircularCW)

%% Setup
clearvars;close all;clc;
addpath(genpath('..\WetiMatlabFunctions'))
addpath(genpath('..\NrelMatlabFunctions'))

% select simulated lidar
LidarType       = '4BeamPulsed'; % [4BeamPulsed/CircularCW]

% simulation time
TMax                = 50; % [s]

% IPC parameters
IPC = [];
IPC.FF.gV = -0.2; %static gain for Vertical component
IPC.FF.gH = 0; %static gain for Horizontal component

switch LidarType
    case '4BeamPulsed'
        % configuration from LDP_v1_4BeamPulsed.IN and FFP_v1_4BeamPulsed.IN
        LDP.NumberOfBeams       = 4;            % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 19.176;       % [deg]     Angle around centerline
        LDP.IndexGate           = 6;            % [-]       IndexGate
        LDP.FlagLPF             = 0;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.1232;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 5.5;          % [s]       Buffer time for filtered REWS signal
        [Y, Z]                  = calculateposlidbeams('LidarFile_4BeamPulsed.dat' , LDP.IndexGate, LDP.NumberOfBeams);
        LDP.Ycoord              = Y;
        LDP.Zcoord              = Z;
        IPC.FB.Kp               = 5.3e-7;
        IPC.FB.Ti               = 4;
    case 'CircularCW'
        % configuration from LDP_v1_CircularCW.IN and FFP_v1_CircularCW.IN
        LDP.NumberOfBeams       = 50;           % [-]       Number of beams measuring at different directions               
        LDP.AngleToCenterline   = 15;           % [deg]     Angle around centerline
        LDP.IndexGate           = 1;            % [-]       IndexGate
        LDP.FlagLPF             = 0;            % [0/1]     Enable low-pass filter (flag)
        LDP.omega_cutoff        = 0.3268;       % [rad/s]   Corner frequency (-3dB) of the low-pass filter
        LDP.T_buffer            = 7.5;          % [s]       Buffer time for filtered REWS signal        
        [Y, Z]                  = calculateposlidbeams('LidarFile_CircularCW.dat' , LDP.IndexGate, LDP.NumberOfBeams);
        LDP.Ycoord              = Y;
        LDP.Zcoord              = Z;
        % Individual pitch controller
        IPC.FB.Kp               = 10.6e-7;
        IPC.FB.Ti               = 8;
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

% phase offset
deltat = 1.1; %s
LDP.deltaphi = R.PC_RefSpd*deltat;

verticalShear       = 0.0167; % [(m/s)/m]

% add FF Parameter from FFP_v1.IN
R.StaticWind        = [0   10.0000   11.0000   12.0000   13.0000   14.0000   15.0000   16.0000   17.0000   18.0000   19.0000   20.0000   21.0000   22.0000   23.0000   24.0000   25.0000   26.0000   27.0000   28.0000   29.0000   30.0000]; % Wind speed  values in static pitch curve [m/s]
R.StaticPitch       = [0         0    0.0552    0.1085    0.1451    0.1749    0.2011    0.2250    0.2473    0.2682    0.2882    0.3072    0.3255    0.3432    0.3603    0.3769    0.3930    0.4087    0.4240    0.4389    0.4535    0.4679]; % Pitch angle values in static pitch curve [rad]

%% Run FB
clear FAST_SFunc 
clear OpenFAST_ROSCO_LDP_FFP
R.FlagLAC           = 0; % Disable LAC
SimOutFB            = sim('OpenFAST_ROSCO_LDP_FFP.slx',[0,TMax]);
movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FB.outb'])      % store results

%% Run FB with IPC
% clear FAST_SFunc 
% clear OpenFAST_ROSCO_LDP_FFP_with_IPC
% R.FlagLAC           = 0; % Disable LAC
% SimOutFB            = sim('OpenFAST_ROSCO_LDP_FFP_with_IPC.slx',[0,TMax]);
% movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FBIPC.outb'])      % store results

%% Run FBFF
clear FAST_SFunc 
clear OpenFAST_ROSCO_LDP_FFP
R.FlagLAC           = 1; % Enable LAC
SimOutFBFF          = sim('OpenFAST_ROSCO_LDP_FFP.slx',[0,TMax]);
movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FBFF.outb'])    % store results

%% Run FBFF with IPC
clear FAST_SFunc 
clear OpenFAST_ROSCO_LDP_FFP_with_IPC
R.FlagLAC           = 1; % Enable LAC
SimOutFBFF          = sim('OpenFAST_ROSCO_LDP_FFP_with_FFIPC.slx',[0,TMax]);
movefile([SimulationName,'.SFunc.outb'],[SimulationName,'_FBFFIPC.outb'])    % store results

%% Comparison
% read in data
FB              = ReadFASTbinaryIntoStruct([SimulationName,'_FB.outb']);
FBFF            = ReadFASTbinaryIntoStruct([SimulationName,'_FBFF.outb']);
% FBIPC           = ReadFASTbinaryIntoStruct([SimulationName,'_FBIPC.outb']);
FBFFIPC         = ReadFASTbinaryIntoStruct([SimulationName,'_FBFFIPC.outb']);

% Plot 
figure('Name','Simulation results')

subplot(4,1,1);
hold on; grid on; box on
plot(FB.Time,       FB.Wind1VelX);
plot(SimOutFBFF.logsout.get('REWS_b').Values);
ylabel('[m/s]');
legend('Wind1VelX','REWS_b','Interpreter','none','Location','best')

subplot(4,1,2);
hold on; grid on; box on
plot(FB.Time,       FB.BldPitch1);
plot(FBFF.Time,     FBFF.BldPitch1);
% plot(FBIPC.Time,     FBIPC.BldPitch1);
plot(FBFFIPC.Time,     FBFFIPC.BldPitch1);
ylabel({'BldPitch1'; '[deg]'});
% legend('feedback only','feedback-feedforward','feedback only with IPC','feedback-feedforward with IPC' ,'Location','best')
legend('feedback only','feedback-feedforward','feedback-feedforward with IPC' ,'Location','best')

subplot(4,1,3);
hold on; grid on; box on
plot(FB.Time,       FB.RotSpeed);
plot(FBFF.Time,     FBFF.RotSpeed);
% plot(FBIPC.Time,     FBIPC.RotSpeed);
plot(FBFFIPC.Time,     FBFFIPC.RotSpeed);
ylabel({'RotSpeed';'[rpm]'});

subplot(4,1,4);
hold on; grid on; box on
plot(FB.Time,       FB.TwrBsMyt/1e3);
plot(FBFF.Time,     FBFF.TwrBsMyt/1e3);
% plot(FBIPC.Time,     FBIPC.TwrBsMyt/1e3);
plot(FBFFIPC.Time,     FBFFIPC.TwrBsMyt/1e3);
ylabel({'TwrBsMyt';'[MNm]'});

xlabel('time [s]')
linkaxes(findobj(gcf, 'Type', 'Axes'),'x');
xlim([20 50])

% figure(4);
% hold on; grid on; box on
% plot(FBFF.Time,     FBFF.RootMyb1);
% plot(FBFF.Time,     FBFF.RootMyb2);
% plot(FBFF.Time,     FBFF.RootMyb3);
% ylabel({'RootMyb'; '[deg]'});
% legend('feedback only','feedback-feedforward','Location','best')
% 
% figure(5);
% hold on; grid on; box on
% plot(FBFF.Time,     FBFF.BldPitch1);
% plot(FBFF.Time,     FBFF.BldPitch2);
% plot(FBFF.Time,     FBFF.BldPitch3);
% ylabel({'RootMyb'; '[deg]'});
% legend('feedback only','feedback-feedforward','Location','best')

%% plot shears
figure;
subplot(2,1,1);
hold on; grid on; box on
plot(FB.Time,       FB.Wind1VelX);
plot(SimOutFBFF.logsout.get('REWS_b').Values);
ylabel('[m/s]');
legend('Wind1VelX','REWS_b','Interpreter','none','Location','northwest')

subplot(2,1,2);
hold on; grid on; box on
plot(SimOutFBFF.logsout.get('deltaV').Values);
plot(SimOutFBFF.logsout.get('deltaV_b').Values);
xlabel('time [s]')
ylabel('Shear [(m/s)/m]')
legend('Vertical Shear','Vertical Shear Buffered','Location','northwest');
% ResizeAndSaveFigure(16,9,'shearResults.fig')

%% display results
RotSpeed_0  = 7.56;     % [rpm]
TwrBsMyt_0  = 158.3e3;  % [kNm]
t_Start     = 0;        % [s]

% Cost_FB = (max(abs(FB.RotSpeed(FB.Time>=t_Start)-RotSpeed_0))) / RotSpeed_0 ...
%      + (max(abs(FB.TwrBsMyt(FB.Time>=t_Start)-TwrBsMyt_0))) / TwrBsMyt_0;

Cost = (max(abs(FBFFIPC.RotSpeed(FBFFIPC.Time>=t_Start)-RotSpeed_0))) / RotSpeed_0 ...
     + (max(abs(FBFFIPC.TwrBsMyt(FBFFIPC.Time>=t_Start)-TwrBsMyt_0))) / TwrBsMyt_0;

% fprintf('Cost for Summer Games 2024 (feedback only) ("30 s sprint"):  %f \n',Cost_FB);
fprintf('Cost for Summer Games 2024 ("FF_IPC"):  %f \n',Cost);