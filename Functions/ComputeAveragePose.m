function TagPose = ComputeAveragePose(TagPoses)
    % ComputeAveragePose approximates average homogenious Transformations 
    % of n Transformation matrices per AprilTag ID.
    %
    % by Joshua Köster
    %
    % To average the Poses, the Transformation Matrix is splitted in the
    % Rotation matrix and the Translation component 
    % the average translation is calculated by the mean values of the
    % vectors
    % averaging the Rotation matrices is not that trivial
    % --> in this case its approximate via Quaternions
    % this is a Method by:
    % Markley, F. Landis, Yang Chen, John Lucas Crassidis, 
    % and Yaakov Oshman. "Average Quaternions." 
    % Journal of Guidance, Control, and Dynamics. 
    % Vol. 30, Issue 4, 2007, pp. 1193-1197.
    %
    
    sizeTags = size(TagPoses,2);
    sizeFrames = size(TagPoses,1);
    Quatcell = cell(sizeFrames,1);
    TransCell = cell(sizeFrames,1);
    for tags = 1:sizeTags
        % first compute the average Rotation via Quaternions
        for frames = 1:sizeFrames
            if validAprilTagPose(TagPoses(frames,tags))
                Quatcell{frames}=PoseRotToQuat(TagPoses(frames,tags));
            end
        end
        % delete double Elememts 
        for frames = 1:sizeFrames-1
            if isequal(Quatcell{frames},Quatcell{frames+1})
                for framesRest = frames+1:sizeFrames
                    if isequal(Quatcell{frames},Quatcell{framesRest})
                        Quatcell{framesRest} = [];
                    end
                end
            end
        end
        % ignore all empty elements
        index = find(~cellfun(@isempty,Quatcell));
        if ~isempty(index)
            for validQuats = 1:length(index)
                Quat(validQuats) = Quatcell{index(validQuats)};
            end 
        end
        if exist('Quat','var')
            avgQuat = meanrot(Quat);
            Rot = quat2rotm(avgQuat);
            
            % now compute average Translation via mean
            % Warning: Visualization in the video might be delayed
            for frames = 1:sizeFrames
                if validAprilTagPose(TagPoses(frames,tags))
                    TransCell{frames} = TagPoses(frames,tags).Trans;
                end
            end
            index = find(~cellfun(@isempty,TransCell));
            for validTransVec = 1:length(index)
                Trans(:,validTransVec) = TransCell{index(validTransVec)};
            end
            meanTrans = mean(Trans,2);
            % combine average Rotation and average Translation in T
            % new Pose gets ID and CornersLoc from the last detected Pose
            T = zeros(4,4); 
            T(1:3,1:3) = Rot;
            T(1:4,4) = e2h(meanTrans);
            TagPose(tags) = AprilTagPose(T,TagPoses(sizeFrames,tags).ID,...
                TagPoses(sizeFrames,tags).TagSize, ...
                TagPoses(sizeFrames,tags).CornersLoc);
        else
            TagPose(tags) = AprilTagPose(zeros(4,4),NaN,0);
        end
    end
end