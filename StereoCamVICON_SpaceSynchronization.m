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
% Script for space synchronizing VICON and Stereo Camera System
% aims the Transformation offset of world coordinateframe from Cam to VICON
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
% (3. Synchronization of the videos
%   --> StereoCamSystem_VideoSynchronization.m)
%
% IMPORTANT: Rotation static status of an Object tracked by VICON unequals 
% Roation static status of AprilTag tracked by a Cam
% --> This script only corrects only the translational component
%
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')
addpath('Data\VICON');


disp('openMaster video...')
[fileMaster,path] =uigetfile('Data\Videos\Master\unsynchron\*.avi');
vMaster = VideoReader([path,fileMaster]);
fps = vMaster.FrameRate;
NumFramesMaster = vMaster.NumFrames;
load('cameraParamsMaster.mat')
cameraParamsMaster = cameraParams;
clear cameraParams

clc
disp('open Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\unsynchron\*.avi');
vSlave = VideoReader([path,fileSlave]);
NumFramesSlave = vSlave.NumFrames;
load('CameraParamsSlave.mat')
cameraParamsSlave = cameraParams;
clear cameraParams

if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

clc
disp('load Calibration of Stereo Camera System')
load(uigetfile('Data\*mat'))
if ~exist('PoseSlaveRelToMaster','var')
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
Abfrage_Tag_size = 'TagSize of the Apriltags (in m): ';
tagSize = input(Abfrage_Tag_size);
Abfrage_ID = 'AprilTag ID: ';
ID = input(Abfrage_ID);
if mod(ID,1) ~= 0 || ID < 0 
    error('ID is always a natural number');
end

clc
abfrage_timeCam = 'Give approximate timestamp in s of StereoCam (static status) (e.g. "1"): ';
TStartCam = input(abfrage_timeCam);

abfrage_timeVICON = 'Give approximate timestamp in s of VICON (static status) (e.g. "1"): ';
TStartVICON = input(abfrage_timeVICON);

clc
abfrage_static = 'How long static status (T in s) (e.g. "5")? ';
staticT = input(abfrage_static);

clc
disp('caluculating rotation and translation offset...')

%% Get Cam Data Static Status

FrameStartCam = round(TStartCam*fps_Cams);
FrameEndCam = FrameStartCam + round(staticT*fps_Cams);

% change here if necessary
Tflipped = troty(180)*trotz(90);
WorldPoseMasterflipped = MultiplyPoses(WorldPoseMaster,Pose(Tflipped));

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
        % change here if necessary
        TagPose = MultiplyPoses(TagPose,Pose(troty(180)));
        TagPoseToWorld = MultiplyPoses(InversePose(WorldPoseMasterflipped),TagPose);
        QuatCamStatic(frames-FrameStartCam+1,1:4) = rotm2quat(TagPoseToWorld.Rot);
        TransCamStatic(frames-FrameStartCam+1,1:3) = TagPoseToWorld.Trans;
    else
        QuatCamStatic(frames-FrameStartCam+1,1:4) = NaN;
        TransCamStatic(frames-FrameStartCam+1,1:3) = NaN;
        infoNaN = infoNaN + 1;
    end
    disp(frames);
end

warning([num2str(infoNaN), ...
    ' AprilTag(s) not deteced']);

%% Get VICON Data Static Status

abfrage_position = 'give position of calibPointer in VICON Data (.csv) (e.g. "1"): ';
pos = input(abfrage_position);
indexstart = 3+(pos-1)*7;   % Important: 7 if Quat angles, 6 if Helix angles

FrameViconStart = round(TStartVICON*fps_VICON) + 3; % first cells Infos
FrameViconEnd = FrameViconStart + round(staticT*fps_VICON);

QuatVICONStatic(:,1) = VICON_Data(FrameViconStart:FrameViconEnd,indexstart+3);
QuatVICONStatic(:,2:4) = VICON_Data(FrameViconStart:FrameViconEnd,indexstart:indexstart+2);
TransVICONStatic = VICON_Data(FrameViconStart:FrameViconEnd,indexstart+4:indexstart+6);

infoNaN = isnan(QuatVICONStatic);
infoNaN = sum(sum(infoNaN));

warning([num2str(infoNaN),' invalid Rotation data of VICON system']);
%%

meanQuatCamStatic = mean(QuatCamStatic);
meanQuatVICONStatic = mean(QuatVICONStatic);
meanTransCamStatic = mean(TransCamStatic);
meanTransVICONStatic = mean(TransVICONStatic./1000);

TCam = zeros(4,4);
TCam(1:3,1:3) = quat2rotm(meanQuatCamStatic);
TCam(1:4,4) = e2h(meanTransCamStatic');
PoseCamStatic = Pose(TCam);
TVICON = zeros(4,4);
TVICON(1:3,1:3) = quat2rotm(meanQuatVICONStatic);
TVICON(1:4,4) = e2h(meanTransVICONStatic');
PoseVICONStatic = Pose(TVICON);

trplot(TCam,'frame','T_{Cam}'); hold on; trplot(TVICON,'frame','T_{VICON}');

% PoseCamRelToVICON = MultiplyPoses(PoseVICONStatic,InversePose(PoseCamStatic));
% PoseCamRelToVICON is irrelevant cause Rotation static status of VICON 
% unequals Roation static status of cam

TCamToVICON = trotx(0);
TCamToVICON(1:3,4) = PoseCamStatic.Trans-PoseVICONStatic.Trans;
WorldPoseCamToVICON = MultiplyPoses(WorldPoseMasterflipped,Pose(TCamToVICON));

PoseCamCorrect = MultiplyPoses(InversePose(WorldPoseCamToVICON),TagPose);
figure;
trplot(PoseCamCorrect.T,'frame','T_{Cam}'); hold on; trplot(TVICON,'frame','T_{VICON}');

Abfrage_name = 'Give name/number for saving calibration data: ';
name = input(Abfrage_name,"s");

save(['Data\CalibrationVICONStereoSystem',name,'.mat'], "PoseSlaveRelToMaster",...
    "WorldPoseMaster", "WorldPoseSlave", "WorldPoseCamToVICON");
