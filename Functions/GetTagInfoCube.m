function CubesPose= GetTagInfoCube(id,pose,CalibratedCubes)
    % GetTagInfoCube callback n AprilTag Poses from n cubes in realtion to
    % their main ID (dorsal) 
    % 
    % by Joshua Köster

    minDiff = 0.4;

    if ~isempty(pose)
       
       for Cubes = 1:size(CalibratedCubes,2)    
           
           if ismember(CalibratedCubes(Cubes).mainID, id)
                T = pose(id == CalibratedCubes(Cubes).mainID).T';
                TagPoseMainID = AprilTagPose(T,CalibratedCubes(Cubes).mainID, ...
                    CalibratedCubes(1,Cubes).PosesToMainID(1,1).TagSize);
           end

           % read out the Poses to their main ID from the detected Tags 
           index = ismember(CalibratedCubes(Cubes).IDs(2:end),id);
           TagPoseToMain = CalibratedCubes(Cubes).PosesToMainID(index);
           
           if size(TagPoseToMain,2) ~= 0
               for DetectedIDs = 1:size(TagPoseToMain,2)
                   if ~ismember(TagPoseToMain(DetectedIDs).ID,id)
                        error('wrong IDs - may be a calibration issue')
                   end
                   % transform the Tag to the MainID Pose
                    T = pose(id == TagPoseToMain(DetectedIDs).ID).T' ...
                       * TagPoseToMain(DetectedIDs).T;
                   CubeTagPoses(DetectedIDs,:) = AprilTagPose(T, ...
                       CalibratedCubes(Cubes).mainID, ...
                       TagPoseToMain(DetectedIDs).TagSize);
    
                   % check if the Transformation is correct
                   if exist('TagPoseMainID','var') && validAprilTagPose(TagPoseMainID)
                       % best and fast case
                       check = PoseSimilarValues(CubeTagPoses(DetectedIDs,:),...
                           TagPoseMainID,minDiff);
                        if ~check && size(TagPoseToMain,2) >= 2
                            % this would be fatal if the MainID pose is
                            % incorrect, so we have to check another ref pose
                                
                            if DetectedIDs == size(TagPoseToMain,2)
                                index = DetectedIDs-1;
                                % IMPORTANT: may be an invalid pose 
                            else
                                index = DetectedIDs+1;
                            end
                            T = pose(id == TagPoseToMain(index).ID).T' ...
                               * TagPoseToMain(index).T;
                            RefPose = AprilTagPose(T, ...
                               CalibratedCubes(Cubes).mainID, ...
                               TagPoseToMain(index).TagSize);
                            checkref = PoseSimilarValues(CubeTagPoses...
                                (DetectedIDs,:),RefPose,minDiff);
                            if ~check && ~checkref
                                
                                warning(['---- ID ',num2str(TagPoseToMain(DetectedIDs).ID),...
                                    ' ---- pose might be incorrect']);
                                % set Pose to invalid
                                CubeTagPoses(DetectedIDs,:) = AprilTagPose...
                                    (zeros(4,4),NaN,CubeTagPoses(DetectedIDs,:).TagSize);
                            else % main ID pose is probably incorrect 
                                warning(['---- MAIN ID ' num2str(CalibratedCubes(Cubes).mainID),...
                                    ' ---- pose might be incorrect']);
                                % set Main ID Pose invalid
                                TagPoseMainID = AprilTagPose(zeros(4,4),NaN,...
                                    CubeTagPoses(DetectedIDs,:).TagSize);
                            end
                        elseif ~check && size(TagPoseToMain,2) == 1
                            warning(['---- CUBE ID ' num2str(CalibratedCubes(Cubes).mainID),...
                                    ' ---- pose might be incorrect']);
                            % set Pose to invalid
                            CubeTagPoses(DetectedIDs,:) = AprilTagPose...
                                (zeros(4,4),NaN,CubeTagPoses(DetectedIDs,:).TagSize);
                            TagPoseMainID = AprilTagPose...
                                (zeros(4,4),NaN,CubeTagPoses(DetectedIDs,:).TagSize);
                        end
                   elseif ~exist('TagPoseMainID','var') && size(TagPoseToMain,2) >= 2
                       if DetectedIDs == size(TagPoseToMain,2)
                            index = DetectedIDs-1;
                            % IMPORTANT: may be an invalid pose 
                       else
                            index = DetectedIDs+1;
                       end
                       T = pose(id == TagPoseToMain(index).ID).T' ...
                           * TagPoseToMain(index).T;
                       RefPose = AprilTagPose(T, ...
                           CalibratedCubes(Cubes).mainID, ...
                           TagPoseToMain(index).TagSize);
                       check = PoseSimilarValues(CubeTagPoses(DetectedIDs,:),...
                            RefPose,minDiff);
                       if ~check 
                           warning(['---- MAIN ID ' num2str(CalibratedCubes(Cubes).mainID),...
                                    ' ---- pose might be incorrect']);
                           CubeTagPoses(DetectedIDs,:) = AprilTagPose...
                               (zeros(4,4),NaN,CubeTagPoses(DetectedIDs,:).TagSize);
                       end
                   end
               end
           end

           if exist('CubeTagPoses','var') && exist('TagPoseMainID','var') ...
                    && validAprilTagPose(TagPoseMainID)
               CubeTagPoses = [CubeTagPoses;TagPoseMainID];
           elseif ~exist('CubeTagPoses','var') && exist('TagPoseMainID','var') ...
                   && validAprilTagPose(TagPoseMainID)
               CubeTagPoses = TagPoseMainID;
           end
          
           if exist('CubeTagPoses','var')
               if exist('CubesPose','var') && isempty(CubesPose)
                   clear CubesPose
               end
               CubesPose(Cubes) = ComputeAveragePose(CubeTagPoses);     
           else
               CubesPose(Cubes) = AprilTagPose(zeros(4,4),NaN,0);
           end
               clear CubeTagPoses TagPoseToMain TagPoseMainID
       end
    else 
        CubesPose = [];
    end
end