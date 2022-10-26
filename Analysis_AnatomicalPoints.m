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
% date: 16.08.2022
%
% Script for read out and analysis anatomical Points
%
% Executed with uEye UI3880CP Cameras
%
% Pre-conditions and hints:
%
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
% 6. Motion Tracking of anatomical points
%   --> StereoCamSystem_MotionCapture_3D.m
%
% 
%% Declaration and initialisation

clear 
close all
clc

addpath('Data')
addpath('Classes')
addpath('Functions')

disp('load anatomical points')
load(uigetfile('Data\*mat'))
if ~exist('AnatomicalPoints','var')
    error('wrong data')
end
T_StartVideoCam = TStart;

clc
disp('load Synchronizaztion of Stereo Camera System')
load(uigetfile('Data\*mat'))
if ~exist("T_Offset",'var')
    error('wrong data')
end

clc
abfrage_deltaT = 'give delta T in s (e.g. 30): ';
delta_T = input(abfrage_deltaT);
deltaFrameCam = round(delta_T*fps_Cams);
deltaFrameVICON = round(delta_T*fps_VICON);

%%  Sorting Cam AprilTag Data

for points = 1:size(AnatomicalPoints,1)
    for frames = 1:deltaFrameCam
        trajectory(:,frames) = AnatomicalPoints{points,frames}.vec;
    end
    field = strcat('pointAprilTag_',num2str(points));
    trajectories.(field) = trajectory;
end

% compute center of points object 
for object = 1:4
    index = 1+(object-1)*4;
    for frames = 1:deltaFrameCam
        for readout = 1:4
            vecsObject(:,readout) = AnatomicalPoints{index+readout-1,frames}.vec;
        end
        vecObject(:,frames) = mean(vecsObject,2);
    end
    field = strcat('objectAprilTag_',num2str(object));
    trajectories.(field) = vecObject;
end



%% XYZ plot

TCam = 1/fps_Cams;      % s

TimeVecCam = [TCam:TCam:delta_T];
TimeVecVICON = [TVICON:TVICON:delta_T];


for object = 1:4
    figure;
    if object == 1
        name = 'Unterschenkel R';
    elseif object == 2
        name = 'Unterschenkel L';
    elseif object == 3
        name = 'Oberschenkel L';
    else 
        name = 'Oberschenkel R';
    end
    subplot(3,1,1);
    plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(1,:),"LineWidth",2); hold on;
    plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(1,:),"LineWidth",2); 
    legend('x_M_S(t)','x_v(t)');
    title(['X Translationskomponente ',name]);
    xlabel('t in s')
    ylabel('x in m')
    set(gca,'Fontsize',25);
    subplot(3,1,2);
    plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(2,:),"LineWidth",2); hold on;
    plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(2,:),"LineWidth",2); 
    legend('y_M_S(t)','y_v(t)');
    title(['Y Translationskomponente ',name])
    xlabel('t in s')
    ylabel('y in m')
    set(gca,'Fontsize',25);
    subplot(3,1,3);
    plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(3,:),"LineWidth",2); hold on;
    plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(3,:),"LineWidth",2); 
    legend('z_M_S(t)','z_v(t)');
    title(['Z Translationskomponente ',name])
    xlabel('t in s')
    ylabel('z in m')
    set(gca,'Fontsize',25);
end

% for object = 1:4
%     figure;
%     plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(1,:),"LineWidth",1.5); hold on;
%     plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(1,:),"LineWidth",1.5); 
%     set(gca,'Fontsize',18);
%     title('X value of anatomical point in world coordinateframe')
%     xlabel('time in s')
%     ylabel('vector X-Value in m')
% end
% 
% for object = 1:4
%     figure;
%     plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(2,:),"LineWidth",1.5); hold on;
%     plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(2,:),"LineWidth",1.5); 
%     set(gca,'Fontsize',18);
%     title('Y value of anatomical point in world coordinateframe')
%     xlabel('time in s')
%     ylabel('vector X-Value in m')
% end
% 
% for object = 1:4
%     figure;
%     plot(TimeVecCam,trajectories.(strcat('objectAprilTag_',num2str(object)))(3,:),"LineWidth",1.5); hold on;
%     plot(TimeVecVICON,trajectories.(strcat('objectVICON_',num2str(object)))(3,:),"LineWidth",1.5); 
%     set(gca,'Fontsize',25);
%     title('Z value of anatomical point in world coordinateframe')
%     xlabel('time in s')
%     ylabel('vector X-Value in m')
% end




%% FFT

FCam = (0:deltaFrameCam-1)*fps_Cams/deltaFrameCam;
FVICON = (0:deltaFrameVICON-1)*fps_VICON/deltaFrameVICON;

for object = 1:4
    for axis = 1:3
        FFTApril(axis,:) = abs(fft(trajectories.(strcat('objectAprilTag_',num2str(object)))(axis,:)));
        FFTVICON(axis,:) = abs(fft(trajectories.(strcat('objectVICON_',num2str(object)))(axis,:)));
    end
end

%% Boxplot Differenz

% first interpolate Kamera AprilTag data
factor = fps_VICON/fps_Cams;
for object = 1:4
    trajec = trajectories.(strcat('objectAprilTag_',num2str(object)));
    for i = 1:3
        trajecinterp(i,:) =interp(trajec(i,:),factor);
    end
    field = strcat('objectAprilTagInterp_',num2str(object));
    trajectories.(field) = trajecinterp;
end

DiffObject1 = trajectories.objectAprilTagInterp_1' - trajectories.objectVICON_1';
DiffObject2 = trajectories.objectAprilTagInterp_2' - trajectories.objectVICON_2';
DiffObject3 = trajectories.objectAprilTagInterp_3' - trajectories.objectVICON_3';
DiffObject4 = trajectories.objectAprilTagInterp_4' - trajectories.objectVICON_4';

X = [DiffObject1(:,1),DiffObject2(:,1),DiffObject3(:,1),DiffObject4(:,1)];
Y = [DiffObject1(:,2),DiffObject2(:,2),DiffObject3(:,2),DiffObject4(:,2)];
Z = [DiffObject1(:,3),DiffObject2(:,2),DiffObject3(:,3),DiffObject4(:,3)];

figure;
boxplot(X,'Labels',{'US R','US L', 'OS L', 'OS R'});
title('Differenz X Translationskomponente')
xlabel('Objekte')
ylabel('x_V(t) - x_M_S(t) (in m)');
figure;
boxplot(Y,'Labels',{'US R','US L', 'OS L', 'OS R'});
title('Differenz Y Translationskomponente')
xlabel('Objekte');
ylabel('y_V(t) - y_M_S(t) (in m)');
figure;
boxplot(Z,'Labels',{'US R','US L', 'OS L', 'OS R'});
title('Differenz Z Translationskomponente')
xlabel('Objekte');
ylabel('z_V(t) - z_M_S(t) (in m)');
