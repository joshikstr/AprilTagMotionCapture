function TagPoses = SavePoseInArray(TagPoses,TagPose,maxFrames)
    % SavePoseInArray saves the Poses of the Tags per frame in a Array
    % --> important for computing the average Pose in the further step
    %
    % by Joshua Köster
    %
    sizePoses = size(TagPoses,1);
    if sizePoses == maxFrames
        TagPoses(1:maxFrames-1,:) = TagPoses(2:maxFrames,:);
        TagPoses(maxFrames,:) = TagPose;
    elseif sizePoses >= 1
        TagPoses(size(TagPoses,1)+1,:) = TagPose;
    else
        TagPoses = TagPose;
    end
end