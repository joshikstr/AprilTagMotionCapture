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
% Script for calibrate Cubes including 5 AprilTags 
%   
% Executed with uEye UI3880CP Camera
%
% Pre-conditions and hints:
%
% Cube with 5 AprilTags --> Main AprilTag on top to describe 
% the Position of the other AprilTags 
% read in a video where every AprilTag is visable in the same frame with
% the main AprilTag
% Cam with a high resolution to detect the AprilTags
% and a low exposure time because of motion blur
% --> Consider adequate lighting
% 
% Scripts before:
% 1. intrinsic Calibration of used Camera
%   --> Camera Calibration App of MATLAB
%
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')

disp('open video...');
[file,path] = uigetfile('Data\Videos\Master\Calibrations\Cube_Calibration\*.avi');
v = VideoReader([path,file]);

disp('open intrinsic calibration data (e.g. cameraParamsMaster)...');
load(uigetfile('Data\*.mat')); 

fps = v.FrameRate;
NumFrames = v.NumFrames;

disp_info = ['The video has framerate of ', num2str(fps), ' Hz with ', ...
    num2str(NumFrames), ' frames'];
disp(disp_info);
pause(2);

Abfrage_Cubes = 'How many Cubes to Calibrate (e.g. "4"): ';
NCubes = input(Abfrage_Cubes);

Abfrage_Durchschnitt = 'How many frames to average? (e.g. "5") ';
nFrames = input(Abfrage_Durchschnitt);
if mod(nFrames,1) ~= 0 || nFrames <= 0 
    error(['average frames must be part ' ...
        'of the natural numbers (without zero)']);
end
Abfrage_TagSize = 'TagSize of the Apriltags (in m) (e.g. "0.06"): ';
tagSize = input(Abfrage_TagSize);

clear Abfrage_Durchschnitt Abfrage_TagSize Abfrage_Cubes

%% main

positions_ID = struct('dorsal_ID',NaN,'medial_ID',NaN,...
    'lateral_ID',NaN,'distal_ID',NaN,'proximal_ID',NaN);
str = ["medial ID: " "lateral ID: " "distal ID: " "proximal ID: "];

for Cubes = 1:NCubes
    clc
    disp(['Ok get ready to calibrate the Cube number ', num2str(Cubes)]);
    pause(3)
    Abfrage_name = 'Name of the Cube: ';
    CubeName = input(Abfrage_name, "s");
    disp('Give the IDs of the AprilTag on the cube')
    Abfrage_IDmain = 'Dorsal ID (main AprilTag): ';
    IDmain = input(Abfrage_IDmain);
    positions_ID.dorsal_ID = IDmain;
    for i = 1:4
        Abfrage_IDs = convertStringsToChars(str(i));
        IDsRemaining(i) = input(Abfrage_IDs);
    end
    positions_ID.medial_ID = IDsRemaining(1);
    positions_ID.lateral_ID = IDsRemaining(2);
    positions_ID.distal_ID = IDsRemaining(3);
    positions_ID.proximal_ID = IDsRemaining(4);
    IDs = [IDmain, IDsRemaining];
    
    
    clear Abfrage_number Abfrage_name Abfrage_IDmain Abfrage_IDs
    
    CalibratedCubes(:,Cubes) = CalibratedAprilTagGroup(IDmain,...
        IDs,positions_ID,Cubes,CubeName);
    TagPose = AprilTagPose(zeros(4,4),NaN,tagSize);
    TagPoses = [];
    
    for TagToCalibrate = 1:length(IDsRemaining)
        clc
        disp(['Calibration of AprilTag ', convertStringsToChars(str(TagToCalibrate)), num2str(IDsRemaining(TagToCalibrate))]);
        Abfrage_Ausschnitt = 'Timestamp of calibration start (in s): ';
        timestamp = input(Abfrage_Ausschnitt);
        IDsTarget = [IDmain,IDsRemaining(TagToCalibrate)];
        disp('calculating...')
    
        frame_start = round(timestamp*fps);
        frame_end = frame_start + nFrames;
        
        clear Abfrage_Ausschnitt
    
        for frames = frame_start:frame_end-1
            image = read(v,frames);
            TagPose = GetTagInfoForCalibration(image, ...
                cameraParams.Intrinsics,tagSize, IDsTarget,[]);
            for tags = 1:length(IDsTarget)
                if ~validAprilTagPose(TagPose(tags))
                    error('Detection of AprilTag(s) failed - check timestamp')
                end
            end
            TagPoses = SavePoseInArray(TagPoses,TagPose,nFrames);
            TagPose = ComputeAveragePose(TagPoses);
        end
        
        % TagPose(1) --> Pose main AprilTag
        % TagPose(2) --> Pose AprilTag to calibrate

        for tags = 1:2
            image = DrawAprilTagPoseInImage(TagPose(tags),image,...
                cameraParams.Intrinsics);
        end
        
        clc
        imshow(image);
        disp('check the image')
        pause(4);
        close gcf
        
        PoseBetweenIDs = MultiplyPoses(InversePose(TagPose(2)),TagPose(1));
        PoseToMain = AprilTagPose(PoseBetweenIDs.T,IDsTarget(2), tagSize);
        CalibratedCubes(:,Cubes).PosesToMainID(:,TagToCalibrate) = PoseToMain;
        disp('succesfull');
        pause(2);
    end
    disp(['Cube number ', num2str(Cubes), ' successfully calibrated']);
    pause(2);
end
clc
save('Data\CalibratedCubes.mat', 'CalibratedCubes');
disp('done and saved!')