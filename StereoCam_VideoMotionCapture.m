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
% Script for capturing anatomical Points in motion realted to AprilTagCubes
% on the Link between two joints on the body (Stereo Cam System)
% visable on the given Master Video
%
% Executed with uEye UI3880CP Camera
%
% Pre-conditions and hints:
%
% First Frame: EVERY or NO Tag must be detected 
% --> important for memory allocation and indices
% Be careful with averaging of the Poses 
% --> consider Filter delay
% Cam with a high resolution to detect the AprilTags
% and a low exposure time because of motion blur
% --> Consider adequate lighting
%
% Scripts before:
% 1. intrinsic Calibration of each Camera
%   --> Camera Calibration App of MATLAB
% 2. extrinsic Calibration of the Stereo Camera System 
%   --> StereoCamSystem_ExtrinsicCalibration.m
% 3. Calibration of the AprilTag Cubes  
%   --> AprilTagCube_Calibration.m
% 4. Calibration of anatomical Points on the body (realated to Cubes) 
%   --> Anatomical_Point_Calibration.m
% 5. Synchronization of the videos
%   --> StereoCamSystem_VideoSynchronization.m
%
% 
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')

disp('open synchronized Master video...')
[fileMaster,path] =uigetfile('Data\Videos\Master\synchron\*.avi');
vMaster = VideoReader([path,fileMaster]);
fps = vMaster.FrameRate; NumFramesMaster = vMaster.NumFrames;
load('cameraParamsMaster.mat')  % load intrinsic Calibration
cameraParamsMaster = cameraParams;

clc
disp('open synchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\synchron\*.avi');
vSlave = VideoReader([path,fileSlave]); NumFramesSlave = vSlave.NumFrames;
load('CameraParamsSlave.mat')   % load intrinsic Calibration
cameraParamsSlave = cameraParams;
clear cameraParams
% check if frame aquisition is synchronized
if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

load('CalibratedCubes.mat')

clc
disp('load Calibration of Stereo Camera System')
load(uigetfile('Data\*mat'))
if ~exist('PoseSlaveRelToMaster','var')
    error('wrong data')
end
clc
disp('load Calibrated Points related to AprilTag Cubes')
load(uigetfile('Data\*mat'))
if ~exist('CalibratedPoints','var')
    error('wrong data')
end

fps = vMaster.FrameRate;
NumFrames = vMaster.NumFrames;

if fps ~= vSlave.FrameRate
    error('wrong video');
end

clc
disp_info = ['The videos have framerate of ', num2str(fps), ...
    ' Hz with ', num2str(NumFrames), ' frames'];
disp(disp_info);
pause(2);

Abfrage_Tag_size = 'TagSize of the Apriltags (in m): ';
tagSize = input(Abfrage_Tag_size);

for dummy = 1:size(CalibratedCubes,2)
    IDsTarget(dummy) = CalibratedCubes(1,dummy).mainID;
end
IDsTarget = sort(IDsTarget);

clc
Abfrage_FrameStart = 'Give frame to start with: ';
firstFrame = input(Abfrage_FrameStart);
if mod(firstFrame,1) ~= 0 || firstFrame <= 0 
    error(['average frames must be part ' ...
        'of the natural numbers (without zero)']);
end
Abfrage_avg_Frames = 'How many frames to average? ';
delta_N = input(Abfrage_avg_Frames);
if mod(delta_N,1) ~= 0 || delta_N <= 0 
    error(['average frames must be part ' ...
        'of the natural numbers (without zero)']);
end

clear disp_info Abfrage_avg_Frames Abfrage_Tag Abfrage_IDs ...
    Abfrage_Tag_size Abfrage_FrameStart dummy 

%% Main

data_name = strcat('MotionCaptureVideos\MotionCaptureVideo', ...
    fileMaster);
v_neu = VideoWriter(data_name,'Motion JPEG AVI');
v_neu.FrameRate = fps;
open(v_neu);
clc
disp('Writing Video...')

TagPose = AprilTagPose(zeros(4,4),NaN,repmat([vMaster.Width/2,vMaster.Height/2],4,1));
TagPoses = [];
CounterPoints = 1;

tic
firstTimeHere= true;

for frames = firstFrame:NumFrames
    imageMaster = read(vMaster,frames);
    imageSlave = read(vSlave,frames);
    TagPose = GetTagInfo(imageMaster,imageSlave,cameraParamsMaster.Intrinsics, cameraParamsSlave.Intrinsics, vMaster, ...
        tagSize, IDsTarget, TagPose, CalibratedCubes, PoseSlaveRelToMaster); 
    if frames == firstFrame && size(TagPose,2) ~= length(IDsTarget)
        error('in the first frame every Cube must be detected')
    end
    if ~isempty(TagPose)
        TagPoses = SavePoseInArray(TagPoses,TagPose,delta_N);
        TagPose = ComputeAveragePose(TagPoses);
        [AnatomicalPoint, imageMaster] = ...
            ComputeVisualAnatomicalPoints(TagPose, CalibratedPoints,...
            imageMaster,cameraParamsMaster.Intrinsics);
        if ~isempty(AnatomicalPoint)
            if frames == firstFrame || ...
                    size(AnatomicalPoint,1)== size(AnatomicalPoints,1)
                AnatomicalPoints(:,CounterPoints) = AnatomicalPoint;
            else    % some Tag is not detected (out of range)
                for detectedPoints = 1:size(AnatomicalPoint,1)
                    AnatomicalPoints{AnatomicalPoint{detectedPoints}.number,...
                        CounterPoints} = AnatomicalPoint{detectedPoints};
                end

            end
            CounterPoints = CounterPoints + 1;
        else
            clear AnatomicalPoint
        end
    end
    writeVideo(v_neu,imageMaster);
    if mod(frames-firstFrame,10) == 0 && frames ~= firstFrame
        if firstTimeHere == false
            disp([num2str(frames-firstFrame),' frames written - ', ...
                num2str(NumFrames-frames), ' frames left to write']);
        else
            elapsedtime = toc;
            estimatedTime = round((elapsedtime/10)*(NumFrames-frames));
            disp(['estimated Time: ',num2str(estimatedTime),' seconds (', ...
                num2str(estimatedTime/60), ' minutes)']);
            disp([num2str(frames-firstFrame),' frames written - ', ...
                num2str(NumFrames-frames), ' frames left to write']);
            firstTimeHere = false;
        end
    end
end
clc
close(v_neu);
disp('done');
Abfrage_name = 'Give name of anatomical points: ';
name = input(Abfrage_name, "s");
save(['Data\AnatomicalPoints',name,'.mat'],"AnatomicalPoints");
disp('...saved!');
