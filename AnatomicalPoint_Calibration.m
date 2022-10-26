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
% Script for calibrate anatomical Points realted to Cube AprilTags on
% the Link between two joints on the body with a calibration pointer
%   
% inspired by:
% Nagymáté, G., and Kiss, R. 2019. Affordable gait analysis using 
% augmented reality markers. PLOS ONE, 14(2).
%
% Executed with uEye UI3880CP Camera
%
% Pre-conditions and hints:
%
% Calibration pointer with a known distance in x to its AprilTag (own ID)
% the pointer must be hold very calmly 
% --> adjust the quantity of frames to average
% Cam with a high resolution to detect the AprilTags
% and a low exposure time because of motion blur
% --> Consider adequate lighting
%
% Scripts before:
% 1. intrinsic Calibration of used Camera
%   --> Camera Calibration App of MATLAB
% 2. Calibration of AprilTag Cubes
%   --> AprilTag_CubeCalibration.m
%
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')

disp('open video...');
[file,path] = uigetfile('Data\Videos\Master\Calibrations\AnatomicalPoints_Calibration\*.avi');
v = VideoReader([path,file]);

clc
disp('open intrinsic calibration data (e.g. cameraParamsMaster)...');
load(uigetfile('Data\*.mat'));
load('CalibratedCubes.mat');

fps = v.FrameRate;
NumFrames = v.NumFrames;

disp_info = ['The video has framerate of ', num2str(fps), ' Hz with ', ...
    num2str(NumFrames), ' frames'];
disp(disp_info);
pause(2);

Abfrage_Punkte = 'How many anatomical points to calibrate? (e.g. "16") ';
N_Points = input(Abfrage_Punkte);
if mod(N_Points,1) ~= 0 || N_Points <= 0 
    error(['quantity of points must be part ...' ...
        'of the natural numbers (without zero)']);
end
Abfrage_Durchschnitt = 'How many frames to average? (e.g. "3") ';
nFrames = input(Abfrage_Durchschnitt);
if mod(nFrames,1) ~= 0 || nFrames <= 0 
    error(['average frames must be part ...' ...
        'of the natural numbers (without zero)']);
end
Abfrage_TagSize = 'TagSize of the Apriltags (in m) (e.g. "0.06"): ';
tagSize = input(Abfrage_TagSize);
Abfrage_Translation = 'Length of the calibration pointer (in m) (e.g. "0.165"): ';
x0 = input(Abfrage_Translation);
Abfrage_ID_pointer = 'AprilTag ID on the calibration pointer (e.g. "0"): ';
IDs = input(Abfrage_ID_pointer);
if mod(IDs,1) ~= 0 || IDs < 0 
    error('ID is always a natural number');
end

clear disp_info Abfrage_Punkte Abfrage_Durchschnitt Abfrage_TagSize ...
    Abfrage_Translation Abfrage_ID_pointer

%% Main

disp('Ok get ready to calibrate the Points...');
pause(3);

TagPose = AprilTagPose(zeros(4,4),NaN,repmat([v.Width/2,v.Height/2],4,1));
TagPoses = [];

for counterPoints = 1:N_Points
    clc
    Abfrage_Ausschnitt = 'Timestamp of calibration start (in s): ';
    timestamp = input(Abfrage_Ausschnitt);
    Abfrage_ID = 'Main AprilTag ID on Link: ';
    IDs(1,2) = input(Abfrage_ID);
    if mod(IDs(1,2),1) ~= 0 || IDs(1,2) < 0 
        error('ID is always a natural number');
    end
    disp('calculating...')

    frame_start = round(timestamp*fps);
    frame_end = frame_start + nFrames;
    
    clear Abfrage_Ausschnitt Abfrage_ID
    
    for frames = frame_start:frame_end-1
        image = read(v,frames);
        TagPose = GetTagInfoForCalibration(image,cameraParams.Intrinsics,tagSize, ...
            IDs,CalibratedCubes);
        if size(TagPose,2) ~= 2
            error('Detection of AprilTag(s) failed - check timestamp')
        end
        TagPoses = SavePoseInArray(TagPoses,TagPose,nFrames);
        TagPose = ComputeAveragePose(TagPoses);
    end
    
    % TagPose(1) --> Pose Calibrator
    % TagPose(2) --> Pose Link

    for tags = 1:2
        image = DrawAprilTagPoseInImage(TagPose(tags),image,...
            cameraParams.Intrinsics);
    end
    
    clc
    imshow(image);
    disp('check the image')
    pause(4);
    %close gcf

    VecCalibrator = Vector([x0;0;0]);
    
    PoseCalibrator2Link = MultiplyPoses(InversePose(TagPose(2)),TagPose(1));
    VecLink = CoordinateTransformation(VecCalibrator,PoseCalibrator2Link);
    CalibratedPoints(:,counterPoints) = ...
        CalibratedVectorAprilTag(VecLink.vec,TagPose(2).ID,counterPoints);
    VecCam = CoordinateTransformation(VecLink,TagPose(2));

    image = DrawPointInImage(VecCam,image,cameraParams.Intrinsics);

    clc
    imshow(image);
    disp('check the image');
    pause(4);
    %close gcf
    clc
    
    VecCamRef = CoordinateTransformation(VecCalibrator,TagPose(1));
    if VecSimilarValues(VecCamRef, VecCam) == true
        disp('The calibration of the point was succesfull');
    else
        warning('something went wrong');
    end
    
    Abfrage_name = 'Name of the calibrated point: ';
    name = input(Abfrage_name, "s");
    CalibratedPoints(:,counterPoints).name = name;
    clear Abfrage_name name
    
    disp_info = [num2str(N_Points-counterPoints), ' point(s) left to calibrate'];
    disp(disp_info);
    clear disp_info
    if counterPoints ~= N_Points
        pause(3);
    end
end

Abfrage_name = 'Give name/number for saving calibration data: ';
name = input(Abfrage_name,"s");

save(['Data\CalibratedPoints',name,'.mat'], 'CalibratedPoints');
clc
disp('...done and saved!')