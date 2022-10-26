function [TagPose, IDs] = GetTagMasterSlave(TagPoseMaster,TagPoseSlave,PoseSlaveRelToMaster)
    % GetTagMasterSlave computes consitant Tag Pose given Tag Poses from
    % differnet cameras 
    % 
    % by Joshua Köster
    
    maxDiff = 0.3;
        
    for dummy=1:size(TagPoseMaster,2)
        IDMaster(dummy) = TagPoseMaster(dummy).ID;
    end
    if exist('IDMaster','var')
        idMaster = IDMaster(~isnan(IDMaster));
    else
        idMaster = [];
    end

    for dummy = 1:size(TagPoseSlave,2)
        IDSlave(dummy) = TagPoseSlave(dummy).ID;
    end
    if exist('IDSlave','var')
        idSlave = IDSlave(~isnan(IDSlave));
    else
        idSlave = [];
    end
    
    if ~isempty(idMaster) && ~isempty(idSlave)
        IDsDetected = union(idMaster,idSlave);
        for ids = 1:length(IDsDetected)
            if ismember(IDsDetected(ids),idMaster) && ...
                    ismember(IDsDetected(ids),idSlave)
                SlaveInMaster = MultiplyPoses(PoseSlaveRelToMaster,...
                    TagPoseSlave(IDsDetected(ids) == IDSlave));
                TagPoses(1,1) = AprilTagPose(SlaveInMaster.T,IDsDetected(ids),...
                    TagPoseSlave(IDsDetected(ids) == IDSlave).TagSize);
                TagPoses(2,1) = TagPoseMaster(IDsDetected(ids) == IDMaster);
                if ~PoseSimilarValues(TagPoses(1,1),TagPoses(2,1),maxDiff)
                    warning(['---- ID ',num2str(IDsDetected(ids)),' ----',...
                        ' Cube Pose might be incorrect']);
                end
                TagPose(ids) = ComputeAveragePose(TagPoses);

            elseif ~ismember(IDsDetected(ids),idMaster) && ...
                    ismember(IDsDetected(ids),idSlave)
                SlaveInMaster = MultiplyPoses(PoseSlaveRelToMaster,...
                    TagPoseSlave(IDsDetected(ids) == IDSlave));
                TagPose(ids) = AprilTagPose(SlaveInMaster.T,IDsDetected(ids),...
                    TagPoseSlave(IDsDetected(ids) == IDSlave).TagSize);

            elseif ismember(IDsDetected(ids),idMaster) && ...
                    ~ismember(IDsDetected(ids),idSlave)
                TagPose(ids) = TagPoseMaster(IDsDetected(ids) == IDMaster);
            else
                error('something went wrong right here')
            end
        end
%         if infoMaster && infoSlave
%             indexMaster = find(isnan(IDMaster));
%             indexSlave = find(isnan(IDSlave));
%             if any(indexMaster == indexSlave)
%                 index = find(indexMaster == indexSlave);
%                 for i = 1:length(index)
%                     TagPose(index(i)) = AprilTagPose(zeros(4,4),NaN, 0);
%                 end
%             end
%         end

    elseif ~isempty(idMaster) && isempty(idSlave)
            TagPose = TagPoseMaster(IDMaster == IDMaster);
    elseif isempty(idMaster) && ~isempty(idSlave)
        for ids = 1:length(idSlave)
            SlaveInMaster = MultiplyPoses(PoseSlaveRelToMaster,...
                TagPoseSlave(idSlave(ids) == IDSlave));
            TagPose(ids) = AprilTagPose(SlaveInMaster.T,idSlave(ids),...
                TagPoseSlave(idSlave(ids) == IDSlave).TagSize);
        end
%         if infoSlave
%             indexSlave = find(isnan(IDSlave));
%             for i = 1:length(indexSlave)
%                 TagPose(indexSlave(i)) = AprilTagPose(zeros(4,4),NaN, 0);
%             end
%         end

    else
        TagPose = [];
    end

    if ~isempty(TagPose)
        for ids = 1:size(TagPose,2)
            IDs(ids) = TagPose(ids).ID;
        end
    else
        IDs = [];
    end
end