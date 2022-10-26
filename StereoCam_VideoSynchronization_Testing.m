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
% 3. Synchronization of the videos
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
fps = vMaster.FrameRate;
NumFramesMaster = vMaster.NumFrames;

clc
disp('open synchronized Slave video...')
[fileSlave,path] =uigetfile('Data\Videos\Slave\synchron\*.avi');
vSlave = VideoReader([path,fileSlave]);
NumFramesSlave = vSlave.NumFrames;

if fps ~= vSlave.FrameRate
    error('Master and Slave must have the same fps')
end

%% Main

while true
    Abfrage_frame = 'give frame (type "0" to exit): ';
    frame = input(Abfrage_frame);
    if frame == 0
        close all
        break
    end
    close all
    fig1 = figure;
    fig1.Position = [0 0 960 1080];
    imageMaster = read(vMaster,frame);
    imshow(imageMaster);
    
    fig2 = figure;
    fig2.Position = [960 0 960 1080];
    imageSlave = read(vSlave,frame);
    imshow(imageSlave);
end