function [CellAnatomicalPoints,image] = ComputeVisualAnatomicalPoints(TagPose, CalibratedPoints, image, Intrinsics)
    % ComputeVisualAnatomicalPoints visualizes the AprilTag Poses and the
    % Anatomical Points as a Vector that are linked to their AprilTags 
    % Furthermore the Function callbacks the Anatomical Points as a
    % AnatomicalPoint Object (referenced to the Cam) for the further 
    % analysis of the points in 3D space
    %
    % by Joshua Köster
    %
    for tags = 1:size(TagPose,2)
        image = DrawAprilTagPoseInImage(TagPose(tags),image,Intrinsics);
        for points = 1:size(CalibratedPoints,2)
            if isVecInAprilTagPose(CalibratedPoints(points),TagPose(tags)) && validAprilTagPose(TagPose(tags))
                VectorCam = CoordinateTransformation(CalibratedPoints(points),TagPose(tags));
                AnatomicalPoint = AnatomicalVectorCam(VectorCam.vec,...
                    VectorCam.number,VectorCam.name);
                image = DrawVectorInImage(AnatomicalPoint,TagPose(tags),image,Intrinsics);
                CellAnatomicalPoints{tags,points} = AnatomicalPoint;
            end
        end
    end
    if exist("CellAnatomicalPoints","var")
         CellAnatomicalPoints = CellAnatomicalPoints(~cellfun('isempty',CellAnatomicalPoints));
    else
        CellAnatomicalPoints = [];
        warning('there are no calibrated points that are linked to the given AprilTag IDs in the image - or there is no valid AprilTagPose in the image')
    end
end