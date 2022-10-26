function TagPose = GetTagInfoForCalibration(image,Intrinsics,...
    tagSize,IDsTarget, CalibratedCubes)
    % GetTagInfoForCalibration detect the AprilTags in the image and 
    % reads in all the information (Pose, Corners Location, ID etc.) 
    % based on the given IDs that have to be analysed 
    %
    % if you want to calibrate Cubes set for CalibratedCubes a empty dummy
    %
    % by Joshua Köster
    
    [id, ~, pose] = readAprilTagModi(image, "tag36h11", Intrinsics, tagSize);
    
    if isempty(id)
        TagPose=[];
    elseif ~isempty(CalibratedCubes)    % Calibration of anatomical Points
        CubesPose = GetTagInfoCube(id,pose,CalibratedCubes);
        for cubes = 1:size(CubesPose,2)
            IDsmainDetected(cubes) = CubesPose(cubes).ID;
        end
        TagPose = AprilTagPose(pose(id == IDsTarget(1)).T',...
            IDsTarget(1), tagSize);   % Calibration pointer
        TagPose(2) = CubesPose(ismember(IDsmainDetected,IDsTarget(2)));
    else % Calibration of Cubes
        if ismember(1,ismember(IDsTarget,id))
            for ids = 1:length(IDsTarget)
                TagPose(ids) = AprilTagPose(pose(id == IDsTarget(ids)).T', ...
                    id(id == IDsTarget(ids)),tagSize);
            end
        else
            warning ('Traget ID not detected, but something strange...')
            TagPose =[];
        end
end