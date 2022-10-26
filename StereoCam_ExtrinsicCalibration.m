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
% Script for Kalibration of Stereo Cam System
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
fps = vMaster.FrameRate;
NumFramesMaster = vMaster.NumFrames;
load('cameraParamsMaster.mat')  % load intrinsic Calibration
cameraParamsMaster = cameraParams;

clc
disp('open unsynchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\unsynchron\*.avi');
vSlave = VideoReader([path,fileSlave]);
NumFramesSlave = vSlave.NumFrames;
load('CameraParamsSlave.mat')   % load intrinsic Calibration
cameraParamsSlave = cameraParams;
clear cameraParams
% check if frame aquisition is synchronized
if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

clc
Abfrage_WS_Master = 'Give frame index worldcoordinate system Master (e.g. "1"): ';
frameMaster = input(Abfrage_WS_Master);
Abfrage_WS_Slave = 'Give frame index worldcoordinate system Slave (e.g. "1"): ';
frameSlave = input(Abfrage_WS_Slave);

Abfrage_avg_Frames = 'How many frames to average? (e.g. "100") ';
nFrames = input(Abfrage_avg_Frames);
if mod(nFrames,1) ~= 0 || nFrames <= 0 
    error(['average frames must be part ' ...
        'of the natural numbers (without zero)']);
end
Abfrage_Tag_size = 'TagSize of the Apriltags (in m) (e.g. "0.06"): ';
tagSize = input(Abfrage_Tag_size);
Abfrage_ID = 'AprilTag traget ID (e.g. "0"): ';
ID = input(Abfrage_ID);
if mod(ID,1) ~= 0 || ID < 0 
    error('ID is always a natural number');
end

clear Abfrage_avg_Frames Abfrage_WS_Slave Abfrage_WS_Master ...
    Abfrage_Tag_size Abfrage_ID

%% compute Relation to worldcoordinate frame
clc
disp('calculationg Poses to worldcoordinate system')

WorldPosesMaster = [];
WorldPosesSlave = [];

% Master to WS
for frames = frameMaster:frameMaster+nFrames
    image = read(vMaster,frames);   % read in image
    WorldPoseMaster = GetTagInfoForCalibration(image,cameraParamsMaster.Intrinsics,...
        tagSize, ID, []);   % compute Pose
    WorldPosesMaster = SavePoseInArray(WorldPosesMaster,WorldPoseMaster,nFrames);
end
WorldPoseMaster = ComputeAveragePose(WorldPosesMaster); % averaging Pose

image = DrawAprilTagPoseInImage(WorldPoseMaster,image,...
            cameraParamsMaster.Intrinsics);
clc
imshow(image);
disp('check the image'); pause(4);
clear image; close gcf

% Slave to WS
for frames = frameSlave:frameSlave+nFrames
    image = read(vSlave,frames);    % read in image
    WorldPoseSlave = GetTagInfoForCalibration(image,cameraParamsSlave.Intrinsics,...
        tagSize, ID, []);   % compute Pose
    WorldPosesSlave = SavePoseInArray(WorldPosesSlave,WorldPoseSlave,nFrames);
end
WorldPoseSlave = ComputeAveragePose(WorldPosesSlave);   % averaging Pose

image = DrawAprilTagPoseInImage(WorldPoseSlave,image,...
            cameraParamsSlave.Intrinsics);
clc
figure; imshow(image);
disp('check the image'); pause(4);
clear image; close gcf; clc;

PoseSlaveRelToMaster = MultiplyPoses(WorldPoseMaster,InversePose(WorldPoseSlave));

baseline = norm(PoseSlaveRelToMaster.Trans);
disp(['triangulation baseline: ', num2str(baseline), 'm']);
Abfrage_baseline = 'realistic? (yes/no): ';
choose = input(Abfrage_baseline, "s");
if choose == "yes"
    disp('perfect')
elseif choose == "no"
    error('triangulation baseline not realistic')
end

Abfrage_name = 'Give name/number for saving calibration data: ';
dataname = ['Data\StereoCamSystem_extrinsicCalibration',input(Abfrage_name,"s"),'.mat'];
save(dataname, "PoseSlaveRelToMaster",...
    "WorldPoseMaster", "WorldPoseSlave");
