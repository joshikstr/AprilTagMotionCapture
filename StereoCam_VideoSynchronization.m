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
% date: 08.08.2022
%
% Script for synchronization of two videos from different cams
% (IMPORTANT: the frame aquisition have to be synchronized) 
%
% Executed with uEye UI3880CP Cameras (Master - Slave PWM Trigger)
%
% Pre-conditions and hints:
% 
% Cams with a high resolution to detect the AprilTags
% and a low exposure time because of motion blur
% --> Consider adequate lighting
% image capturing of the cams have to be synchronized
%
% Scripts before:
% 1. intrinsic Calibration of each Camera
%   --> Camera Calibration App of MATLAB
% 2. Stereo Camera System extrinsic Calibration
%   --> StereoCamSystem_ExtrinsicCalibration.m
%
%
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')

disp('open unsynchronized Master video...')
[fileMaster,path] =uigetfile('Data\Videos\Master\unsynchron\*.avi');
vMaster = VideoReader([path,fileMaster]);
fps = vMaster.FrameRate; NumFramesMaster = vMaster.NumFrames;
load('cameraParamsMaster.mat')  % load intrinsic Calibration
cameraParamsMaster = cameraParams;

clc
disp('open unsynchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\unsynchron\*.avi');
vSlave = VideoReader([path,fileSlave]); NumFramesSlave = vSlave.NumFrames;
load('CameraParamsSlave.mat')   % load intrinsic Calibration
cameraParamsSlave = cameraParams;
clear cameraParams
% check if frame aquisition is synchronized
if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

clc
disp('load extrinsic Calibration of Stereo Camera System')
load(uigetfile('Data\*mat'))
if ~exist('PoseSlaveRelToMaster','var')
    error('wrong data')
end

clc
Abfrage_Tag_size = 'TagSize of the Apriltags (in m) (e.g. "0.06"): ';
tagSize = input(Abfrage_Tag_size);
Abfrage_ID = 'AprilTag ID: ';
ID = input(Abfrage_ID);
if mod(ID,1) ~= 0 || ID < 0 
    error('ID is always a natural number');
end

Abfrage_frameStart = 'frame in Slave to start with the cross correlation (e.g. "300"): ';
frameStart = input(Abfrage_frameStart);
if mod(frameStart,1) ~= 0 || frameStart < 0 
    error('frame is always a natural number');
end

Abfrage_frames = 'how many frames cross correlation? (e.g. "550") ';
deltaFrames = input(Abfrage_frames);
if mod(deltaFrames,1) ~= 0 || deltaFrames < 0 
    error('frame is always a natural number');
end
clear Abfrage_frames Abfrage_frameStart Abfrage_Tag_size Abfrage_ID

%% main (compute time offset (Important: Master Video first started to capture))

clc
disp('calculating...')

% Master 
counter = 1;
frameEnd = frameStart+deltaFrames-1;
for frames = frameStart:frameEnd
    image = read(vMaster,frames);   % read in image
    TagPose = GetTagInfoForCalibration(image,cameraParamsMaster.Intrinsics,...
        tagSize,ID,[]);
    if ~isempty(TagPose)
        TagVec = Vector(TagPose.Trans);
        VecWorld = CoordinateTransformation(TagVec,InversePose(WorldPoseMaster));
        VecWorldMaster(:,counter) = VecWorld.vec;
        counter = counter + 1;
    end
end
warning([num2str(deltaFrames - counter+1), ...
    ' AprilTag(s) of Master not deteced']);

clear TagPose TagVec VecWorld

% Slave 
counter = 1;
frameEnd = frameStart+deltaFrames-1;
for frames = frameStart:frameEnd
    image = read(vSlave,frames);    % read in image
    TagPose = GetTagInfoForCalibration(image,cameraParamsSlave.Intrinsics,...
        tagSize,ID,[]);
    if ~isempty(TagPose)
        TagVec = Vector(TagPose.Trans);
        VecWorld = CoordinateTransformation(TagVec,InversePose(WorldPoseSlave));
        VecWorldSlave(:,counter) = VecWorld.vec;
        counter = counter + 1;
    end
end

warning([num2str(deltaFrames - counter+1), ...
    ' AprilTag(s) of Slave not deteced']);
pause(3);

framesVec = [frameStart:frameStart+deltaFrames-1];
figure;
subplot(3,1,1);
plot(framesVec,VecWorldMaster(1,:),'LineWidth',2); 
hold on; plot(framesVec,VecWorldSlave(1,:),'LineWidth',2);
title('X Translationskomponenten');
xlabel('f (einheitenlos)');
ylabel('X in m');
legend('x_M(f)','x_S(f)');
subplot(3,1,2);
plot(framesVec,VecWorldMaster(2,:),'LineWidth',2); 
hold on; plot(framesVec,VecWorldSlave(2,:),'LineWidth',2);
title('Y Translationskomponenten');
xlabel('f (einheitenlos)');
ylabel('Y in m');
legend('y_M(f)','y_S(f)');
subplot(3,1,3);
plot(framesVec,VecWorldMaster(3,:),'LineWidth',2); 
hold on; plot(framesVec,VecWorldSlave(3,:),'LineWidth',2);
title('Z Translationskomponenten');
xlabel('f (einheitenlos)');
ylabel('Z in m');
legend('z_M(f)','z_S(f)');

maxlag = 100; % Slave Start max 5s after Master Start

[xCorr_VecX,LagX] = xcorr(VecWorldMaster(1,:),VecWorldSlave(1,:),maxlag);
[xCorr_VecY,LagY] = xcorr(VecWorldMaster(2,:),VecWorldSlave(2,:),maxlag);
[xCorr_VecZ,LagZ] = xcorr(VecWorldMaster(3,:),VecWorldSlave(3,:),maxlag);
figure;
plot(LagX,xCorr_VecX); hold on; plot(LagY,xCorr_VecY); plot(LagZ,xCorr_VecZ); hold off;

[~,frameX] = max(xCorr_VecX);
[~,frameY] = max(xCorr_VecY);
[~,frameZ] = max(xCorr_VecZ);
frameX = LagX(frameX);
frameY = LagY(frameY);
frameZ = LagZ(frameZ);

if frameX == frameY && frameY == frameZ
    clc
    disp('great Job!');
    frameOffset = frameX-size(VecWorldMaster,2);
    disp(['frame Offset: ', num2str(frameOffset)])
else
    clc
    disp(['max value of cross correlation in X direction at frame ', ...
        num2str(frameX)]);
    disp(['max value of cross correlation in Y direction at frame ', ...
        num2str(frameY)]);
    disp(['max value of cross correlation in Z direction at frame ', ...
        num2str(frameZ)]);
    Abfrage_which = 'wich direction do you trust? (x/y/z) or manuel input (m) ';
    choose = input(Abfrage_which,"s");
    if choose == "x"
        frameOffset = frameX;
    elseif choose == "y"
        frameOffset = frameY;
    elseif choose == "z"
        frameOffset = frameZ;
    elseif choose == "m"
        abfrage_frameoffset = 'give value: ';
        frameOffset = input(abfrage_frameoffset);
    else
        error('you have to decide')
    end
end
clear Abfrage_which

%% write new Videos

Abfrage_frameStart = 'Give start frame to write (end of synchronization) (e.g. "1000): ';
frameStartWrite = input(Abfrage_frameStart);
if mod(frameStartWrite,1) ~= 0 || frameStartWrite <= 0 
    error(['frames must be part ' ...
        'of the natural numbers (without zero)']);
end
clear Abfrage_frameStart

vMasterNeu = VideoWriter(['Data\Videos\Master\synchron\',fileMaster], 'Motion JPEG AVI');
vSlaveNeu = VideoWriter(['Data\Videos\Slave\synchron\',fileSlave], 'Motion JPEG AVI');

vMasterNeu.FrameRate = fps; 
vSlaveNeu.FrameRate = fps;
open(vMasterNeu); 
open(vSlaveNeu);

disp('Writing Videos...')

for frames = frameStartWrite:NumFramesMaster-frameOffset
    tic
    image = read(vMaster,frames+frameOffset);
    writeVideo(vMasterNeu,image);
    image = read(vSlave,frames);
    writeVideo(vSlaveNeu,image);
    if mod(frames-frameStartWrite,50) == 0 && frames ~= frameStartWrite
        disp([num2str(frames-frameStartWrite),' frames written - ', ...
            num2str(NumFramesMaster-frameOffset-frames), ' frames left to write']);
    end
    if frames == frameStartWrite
        elapsedtime = toc;
        estimatedTime = round(elapsedtime*(NumFramesMaster-frameOffset-frameStartWrite));
        disp(['estimated Time: ',num2str(estimatedTime),' seconds (', ...
            num2str(estimatedTime/60), ' minutes)']);
    end
end

close(vMasterNeu);
close(vSlaveNeu);
clc
disp('...done and saved!')
