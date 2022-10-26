% Motion Capture System based on AprilTags
% 
% Bachelor thesis 
% Joshua Köster
% MatrNr.: 17201828
% Fachhochschule Dortmund - University of Applied Science and Arts Dortmund
% faculty: Informationstechnik - information technology 
% course of studies: Biomedizintechnik - biomedical technology
% e-mail: joshua.koester011@stud.fh-dortmund.de
% supervisor: Prof. Dr.-Ing Jörg Thiem
% date: 23.08.2022
%
% Script for time synchronizing VICON and Stereo Camera System
% aims the time offset between the VICON and Cam Start
%
% Executed with uEye UI3880CP Cameras
%
% Pre-conditions and hints:
%
% Note that the VICON and stereo cam system might be have a differnet fps
% in THIS case 
% Cam: 20 Hz 
% VICON: 300 Hz
% --> important to compute the offset in seconds
% Cam with a high resolution to detect the AprilTags
% and a low exposure time because of motion blur
% --> Consider adequate lighting
% 
% Scripts before:
% 1. intrinsic Calibration of each Camera
%   --> Camera Calibration App of MATLAB
% 2. Stereo Camera System extrinsic Calibration
%   --> StereoCamSystem_ExtrinsicCalibration.m
% 3. Synchronization of the videos
%   --> StereoCamSystem_VideoSynchronization.m
% 4. Space Synchronization of StereoCam and VICON
%   --> VICON_StereoCamSystem_SpaceSynchronization.m
%
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')
addpath('Data\VICON');


disp('open synchronized Master video...')
[fileMaster,path] =uigetfile('Data\Videos\Master\synchron\*.avi');
vMaster = VideoReader([path,fileMaster]);
fps = vMaster.FrameRate;
NumFramesMaster = vMaster.NumFrames;
load('cameraParamsMaster.mat')
cameraParamsMaster = cameraParams;
clear cameraParams

clc
disp('open synchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\synchron\*.avi');
vSlave = VideoReader([path,fileSlave]);
NumFramesSlave = vSlave.NumFrames;
load('CameraParamsSlave.mat')
cameraParamsSlave = cameraParams;
clear cameraParams

if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

clc
disp('load VICON Calibration of Stereo Camera System')
load(uigetfile('Data\*mat'))
if ~exist("WorldPoseCamToVICON",'var')
    error('wrong data')
end

clc
disp('open VICON data...')
fileVICON =uigetfile('Data\VICON\*.csv');
VICON_Data = readmatrix(fileVICON);
fps_VICON = 300;      % Hz
fps_Cams = fps;       % Hz
clear fps

clc
Abfrage_Tag_size = 'TagSize of the Apriltags (in m) (e.g. "0,06"): ';
tagSize = input(Abfrage_Tag_size);
Abfrage_ID = 'AprilTag ID: ';
ID = input(Abfrage_ID);
if mod(ID,1) ~= 0 || ID < 0 
    error('ID is always a natural number');
end

clc
abfrage_timeCam = 'Give approximate timestamp in s of StereoCam (e.g. "15"):';
TStartCam = input(abfrage_timeCam);

abfrage_timeVICON = 'Give approximate timestamp in s of VICON (e.g. "2"): ';
TStartVICON = input(abfrage_timeVICON);

clc
abfrage_TimeAnalysis = 'How long analysis (T in s) (e.g. "30"): ';
deltaT = input(abfrage_TimeAnalysis);

%% Get Data from Cams
clc
disp('calculating...')
FrameStartCam = round(TStartCam*fps_Cams);
FrameEndCam = FrameStartCam + round(deltaT*fps_Cams);

infoNaN = 0;
for frames = FrameStartCam:FrameEndCam
    imageMaster = read(vMaster,frames);
    imageSlave = read(vSlave,frames);
    [id,~,pose] = readAprilTagModi(imageMaster,"tag36h11",...
        cameraParamsMaster.Intrinsics,tagSize);
    if ismember(ID,id)
        TagPoseMaster= AprilTagPose(pose(id==ID).T',ID,tagSize);
    else
        TagPoseMaster = [];
    end

    [id,~,pose] = readAprilTagModi(imageSlave,"tag36h11",...
        cameraParamsSlave.Intrinsics,tagSize);
    if ismember(ID,id)
        TagPoseSlave = AprilTagPose(pose(id==ID).T',ID,tagSize);
    else
        TagPoseSlave = [];
    end
    
    TagPose = GetTagMasterSlave(TagPoseMaster,TagPoseSlave,PoseSlaveRelToMaster);
    
    if ~isempty(TagPose)
        % change if 
        TagPose = MultiplyPoses(TagPose,Pose(troty(180)));
        TagPoseToWorld = MultiplyPoses(InversePose(WorldPoseCamToVICON),TagPose);
        QuatCam(frames-FrameStartCam+1,1:4) = rotm2quat(TagPoseToWorld.Rot);
        TransCam(frames-FrameStartCam+1,1:3) = TagPoseToWorld.Trans;
    else
        QuatCam(frames-FrameStartCam+1,1:3) = NaN;
        TransCam(frames-FrameStartCam+1,1:3) = NaN;
        infoNaN = infoNaN + 1;
    end
    disp(frames);
end

warning([num2str(infoNaN),' AprilTag(s) not deteced']);

%% Get Data from VICON

abfrage_position = 'give position of calibPointer in VICON Data (.csv) (e.g. "1"): ';
pos = input(abfrage_position);
indexstart = 3+(pos-1)*7;   % Important: 7 if Quat angles, 6 if Helix angles

FrameViconStart = round(TStartVICON*fps_VICON) + 3; % first cells Infos
FrameViconEnd = FrameViconStart + round(deltaT*fps_VICON);

QuatVICON(:,1) = VICON_Data(FrameViconStart:FrameViconEnd,indexstart+3);
QuatVICON(:,2:4) = VICON_Data(FrameViconStart:FrameViconEnd,indexstart:indexstart+2);
TransVICON = VICON_Data(FrameViconStart:FrameViconEnd,indexstart+4:indexstart+6);

infoNaN = isnan(QuatVICON);
infoNaN = sum(sum(infoNaN));

warning([num2str(infoNaN),' invalid Rotation data of VICON system']);

%% manipulate

meanQuatCamstatic = mean(QuatCam(1:10,:));      % average over 0,5s
meanRotmCamstatic = quat2rotm(meanQuatCamstatic);

meanQuatVICONstatic = mean(QuatVICON(1:150,:)); % average over 0,5s
meanRotmVICONstatic = quat2rotm(meanQuatVICONstatic);
RotmCamRelToVICON = meanRotmVICONstatic^(-1) * meanRotmCamstatic;
RotmVICON = cell(length(QuatVICON),1);
for dummy = 1:length(QuatVICON)
    RotmVICON{dummy} = quat2rotm(QuatVICON(dummy,:));
    RotmVICON{dummy} = RotmVICON{dummy} * RotmCamRelToVICON;
    QuatVICON(dummy,:) = rotm2quat(RotmVICON{dummy});
end


%% Plot Data

T_OffsetApprox = TStartVICON - TStartCam;

% T_Cam = [TStartCam:1/fps_Cams:TStartCam+deltaT];
T_VICON = [TStartVICON-T_OffsetApprox:1/fps_VICON:TStartVICON-T_OffsetApprox+deltaT];

% Interpolation Euler Angles from cam, cause F_cam ~= F_vicon
% important for cross correlation

factor = fps_VICON/fps_Cams;
for i = 1:3
    TransCamMM(:,i) = TransCam(:,i).*1000;
    TransCamInterp(:,i) = interp(TransCamMM(:,i),factor);
end
for i = 1:4
    QuatCamInterp(:,i) = interp(QuatCam(:,i),factor);
end

diff = size(QuatCamInterp,1)-length(T_VICON);
QuatCamInterp = QuatCamInterp(1:end-diff,:);
TransCamInterp = TransCamInterp(1:end-diff,:);

figure;
subplot(4,1,1);
plot(T_VICON,QuatCamInterp(:,1),'LineWidth',2); hold on; 
plot(T_VICON,QuatVICON(:,1),'LineWidth',2);
title('Quat Realteil');
xlabel('t in s');
ylabel('s (o.E.)');
legend('s_K(t)','s_V(t)');
subplot(4,1,2);
plot(T_VICON,QuatCamInterp(:,2),'LineWidth',2); hold on; 
plot(T_VICON,QuatVICON(:,2),'LineWidth',2);
title('Quat Imaginärteil');
xlabel('t in s');
ylabel('v_1 (o.E.)');
legend('v_1_K(t)','v_1_V(t)');
subplot(4,1,3);
plot(T_VICON,QuatCamInterp(:,3),'LineWidth',2); hold on; 
plot(T_VICON,QuatVICON(:,3),'LineWidth',2);
title('Quat Imaginärteil');
xlabel('t in s');
ylabel('v_2 (o.E.)');
legend('v_2_K(t)','v_2_V(t)');
subplot(4,1,4);
plot(T_VICON,QuatCamInterp(:,4),'LineWidth',2); hold on; 
plot(T_VICON,QuatVICON(:,4),'LineWidth',2);
title('Quat Imaginärteil');
xlabel('t in s');
ylabel('v_3 (o.E.)');
legend('v_3_K(t)','v_3_V(t)');

figure;
subplot(3,1,1);
plot(T_VICON,TransCamInterp(:,1),'LineWidth',2); hold on; 
plot(T_VICON,TransVICON(:,1),'LineWidth',2);
title('X Translationskomponente');
xlabel('t in s');
ylabel('X in mm');
legend('x_K(t)','x_V(t)');
subplot(3,1,2);
plot(T_VICON,TransCamInterp(:,2),'LineWidth',2); hold on; 
plot(T_VICON,TransVICON(:,2),'LineWidth',2);
title('Y Translationskomponente');
xlabel('t in s');
ylabel('X in mm');
legend('y_K(t)','y_V(t)');
subplot(3,1,3);
plot(T_VICON,TransCamInterp(:,3),'LineWidth',2); hold on; 
plot(T_VICON,TransVICON(:,3),'LineWidth',2);
title('Z Translationskomponente');
xlabel('t in s');
ylabel('Z in mm');
legend('z_K(t)','z_V(t)');


%% cross correlation
abfrage_timecross = 'give epected range offset (T in s): ';
expctOffset = input(abfrage_timecross);
frameCross = round(expctOffset*fps_VICON);

[xCorrRw, Lag] = xcorr(QuatVICON(:,1),QuatCamInterp(:,1),frameCross);
xCorrRx = xcorr(QuatVICON(:,2),QuatCamInterp(:,2),frameCross);
xCorrRy = xcorr(QuatVICON(:,3),QuatCamInterp(:,3),frameCross);
xCorrRz = xcorr(QuatVICON(:,4),QuatCamInterp(:,4),frameCross);

T_corr = Lag.*1/fps_VICON;

figure;
plot(T_corr,xCorrRw); hold on;
plot(T_corr,xCorrRx)
plot(T_corr,xCorrRy)
plot(T_corr,xCorrRz)
title('cross correlation of Quat Angles')

[xCorrTX, Lag] = xcorr(TransVICON(:,1),TransCamInterp(:,1),frameCross);
xCorrTY = xcorr(TransVICON(:,2),TransCamInterp(:,2),frameCross);
xCorrTZ = xcorr(TransVICON(:,3),TransCamInterp(:,3),frameCross);

figure;
plot(T_corr,xCorrTX); hold on;
plot(T_corr,xCorrTY)
plot(T_corr,xCorrTZ)
title('cross correlation of Translation')
%%
abfrage_choose = 'FURTHER PROCEED: 1. set manual offset -> type "m" 2. analysis maximums of cross correlation -> type "c"   ';
choose = input(abfrage_choose,"s");
if choose == "m"
    clc
    abfrage_offset = 'ok, give time offset (delta_T = T_Cam - T_VICON in s): ';
    T_Offset = T_OffsetApprox + input(abfrage_offset);
elseif choose == "c"
    clc
    abfrage_crossStart = 'ok, give range start (T_Start in S) (e.g. "2"): ';
    crossStart = input(abfrage_crossStart);
    crossStartFrames = frameCross + round(crossStart*fps_VICON);
    abfrage_crossEnd = 'now give range end (T_End in S) (e.g. "3"): ';
    crossEnd = input(abfrage_crossEnd);
    crossEndFrames = frameCross + round(crossEnd*fps_VICON);

    [~,frameTx] = max(xCorrTX(crossStartFrames:crossEndFrames));
    [~,frameTy] = max(xCorrTY(crossStartFrames:crossEndFrames));
    [~,frameTz] = max(xCorrTZ(crossStartFrames:crossEndFrames));

    [~,frameRw] = max(xCorrRw(crossStartFrames:crossEndFrames));
    [~,frameRx] = max(xCorrRx(crossStartFrames:crossEndFrames));
    [~,frameRy] = max(xCorrRy(crossStartFrames:crossEndFrames));
    [~,frameRz] = max(xCorrRz(crossStartFrames:crossEndFrames));

    % convert to seconds
    T_OffsetTx = (crossStartFrames+frameTx-frameCross+1)/fps_VICON;
    T_OffsetTy = (crossStartFrames+frameTy-frameCross+1)/fps_VICON;
    T_OffsetTz = (crossStartFrames+frameTz-frameCross+1)/fps_VICON;

    T_OffsetRw = (crossStartFrames+frameRw-frameCross+1)/fps_VICON;
    T_OffsetRx = (crossStartFrames+frameRx-frameCross+1)/fps_VICON;
    T_OffsetRy = (crossStartFrames+frameRy-frameCross+1)/fps_VICON;
    T_OffsetRz = (crossStartFrames+frameRz-frameCross+1)/fps_VICON;

    disp(['Tx: ',num2str(T_OffsetTx),' s']);
    disp(['Ty: ',num2str(T_OffsetTy),' s']);
    disp(['Tz: ',num2str(T_OffsetTz),' s']);
    disp(['Rw: ',num2str(T_OffsetRw),' s']);
    disp(['Rx: ',num2str(T_OffsetRx),' s']);
    disp(['Ry: ',num2str(T_OffsetRy),' s']);
    disp(['Rz: ',num2str(T_OffsetRz),' s']);
    
    abfrage_offset = 'give time offset (delta T in s): ';
    T_Offset = T_OffsetApprox + input(abfrage_offset);
else
    error('you have to decide')
end

%% plot
T_Cam = [TStartCam:1/fps_Cams:TStartCam+deltaT];
T_VICON = [TStartVICON-T_Offset:1/fps_VICON:TStartVICON-T_Offset+deltaT];

% Interpolation Euler Angles from cam, cause F_cam ~= F_vicon
% important for cross correlation
abfrage_plot = 'want to see the plot? (y/n): ';
choose = input(abfrage_plot,"s");

if choose == "y"
    figure;
    subplot(4,1,1);
    plot(T_Cam,QuatCam(:,1)); hold on; 
    plot(T_VICON,QuatVICON(:,1));
    title('Rw');
    subplot(4,1,2);
    plot(T_Cam,QuatCam(:,2)); hold on; 
    plot(T_VICON,QuatVICON(:,2));
    title('Rx');
    subplot(4,1,3);
    plot(T_Cam,QuatCam(:,3)); hold on; 
    plot(T_VICON,QuatVICON(:,3));
    title('Ry');
    subplot(4,1,4);
    plot(T_Cam,QuatCam(:,4)); hold on; 
    plot(T_VICON,QuatVICON(:,4));
    title('Rz');
    
    figure;
    subplot(3,1,1);
    plot(T_Cam,TransCamMM(:,1)); hold on; 
    plot(T_VICON,TransVICON(:,1));
    title('Tx');
    subplot(3,1,2);
    plot(T_Cam,TransCamMM(:,2)); hold on; 
    plot(T_VICON,TransVICON(:,2));
    title('Ty');
    subplot(3,1,3);
    plot(T_Cam,TransCamMM(:,3)); hold on; 
    plot(T_VICON,TransVICON(:,3));
    title('Tz');
end


%%

Abfrage_name = 'Give name/number for saving data: ';
name = input(Abfrage_name,"s");

save(['Data\VICONStereoSystemSynchronization',name,'.mat'], "PoseSlaveRelToMaster",...
    "WorldPoseMaster", "WorldPoseSlave", "WorldPoseCamToVICON", "T_Offset");