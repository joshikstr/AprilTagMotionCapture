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
% Script for manually synchronization of two videos from different cams
% Run only if he automatic Synchronization is incorrect
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
% 3. Synchronization of the videos
%   --> StereoCamSystem_VideoSynchronization.m
% 4. testing Synchronization of the videos
%   --> StereoCamSystem_VideoSynchronization_Testing.m
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

clc
disp('open unsynchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\unsynchron\*.avi');
vSlave = VideoReader([path,fileSlave]);
NumFramesSlave = vSlave.NumFrames;

if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

%% main 

abfrage_frameoffset = 'give frame offset value: ';
frameOffset = input(abfrage_frameoffset);

Abfrage_frameStart = 'Give start frame to write (end of synchronization): ';
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