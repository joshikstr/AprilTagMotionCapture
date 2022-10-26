function CubePose = GetTagInfo(imageMaster,imageSlave,IntrinsicsMaster, IntrinsicsSlave,VideoObject,...
    tagSize,IDsTarget, CubePoseBefore, CalibratedCubes, PoseSlaveRelToMaster)
    % GetTagInfo detect the AprilTags in the image and reads in all the 
    % information (Pose, Corners Location, ID etc.) based on the given main 
    % IDs of the cubes that have to be analysed  
    %
    % by Joshua Köster
    %
    % There are four possible cases that can occur
    % INFO : Cubes = calibrated AprilTagGroup
    %
    % 1. the expected Cubes were detected (best case)
    % 
    % 2. there are no detected Cubes in the image 
    % (might be no Tag in the image - bad case if only one Tag is used)
    %
    % Warning:  There is a rare case that more than 
    %           two Cubes are not detected in a image 
    %
    % 3. There are more detected Cubes than expected (rare but not incommon)
    % -> only read in the Poses from the target IDs
    %   
    % 4. Less detected Tags than expected (cirtical case)
    % 
    % There are two options why at least one Tag is not deteced
    %
    % 4a.   The Tag is no longer in the Image
    % -->   to verify check the TagPose Array from the frame before
    %       maybe the Tag was in a border range (5% of the image edges)
    %       in this case the AprilTagPose is not valid
    %
    % 4b.   For several reases the Tag is not detected but it is not in
    %       the border range (often the Tag is blurred etc.)
    % -->   Get the Pose of the frame before
    %       Depending on the Framerate the aproximation gets dangerous
    %       for more than 5-10 frames
    %
    % Note that case 3 is managed in GetTagInfoCube
    %
    % Warnung:  there is a rare case, that a not detected Tag is still
    %           in the image, but the algorithm says its not
    
    % get Poses Master
%     [idOrig, ~, poseOrig] = readAprilTag(imageMaster, "tag36h11", ...
%         IntrinsicsMaster, tagSize);
%     [idBright, ~, poseBright] = readAprilTag(imlocalbrighten(imageMaster), ...
%         "tag36h11", IntrinsicsMaster, tagSize);
% 
%     id = union(idOrig,idBright);
%     for i = 1:length(id)
%         if ismember(id(i),idOrig)
%             pose(i) = poseOrig(id(i)==idOrig);
%         else
%             pose(i) = poseBright(id(i)==idBright);
%         end
%     end
    
    [id, ~, pose] = readAprilTagModi(imageMaster, "tag36h11", ...
         IntrinsicsMaster, tagSize);

    % get CubePose Master
    if ~isempty(id)
        MasterCubePose = GetTagInfoCube(id,pose,CalibratedCubes);
    else
        MasterCubePose = [];
    end


  
%     % get Pose Slave
%     [idOrig, ~, poseOrig] = readAprilTag(imageSlave, "tag36h11", ...
%         IntrinsicsSlave, tagSize);
%     [idBright, ~, poseBright] = readAprilTag(imlocalbrighten(imageSlave), ...
%         "tag36h11", IntrinsicsSlave, tagSize);
% 
%     id = union(idOrig,idBright);
%     for i = 1:length(id)
%         if ismember(id(i),idOrig)
%             pose(i) = poseOrig(id(i)==idOrig);
%         else
%             pose(i) = poseBright(id(i)==idBright);
%         end
%     end
    
    [id, ~, pose] = readAprilTagModi(imageSlave, "tag36h11", ...
         IntrinsicsSlave, tagSize);

    % get CubePose Slave
    if ~isempty(id)
        SlaveCubePose = GetTagInfoCube(id,pose,CalibratedCubes);
    else
        SlaveCubePose = [];
    end

    if isempty(SlaveCubePose) && isempty(MasterCubePose)
        IDmain = [];
    else
        [DetectedCubePose, IDmain] = GetTagMasterSlave(MasterCubePose,...
        SlaveCubePose, PoseSlaveRelToMaster);
    end
    
    if ~isempty(IDmain)
        % set NaN ID values for not detected Cubes
        % important for averaging poses per frame later
        invalidPose = AprilTagPose(zeros(4,4),NaN, tagSize);
        if length(IDmain) ~= length(IDsTarget)
            for ids = 1:length(IDsTarget)
                if ~ismember(IDsTarget(ids),IDmain)
                    if ids == 1
                        IDmain = [NaN,IDmain];
                        DetectedCubePose = [invalidPose,DetectedCubePose];
                    elseif ids > 1 && ids < length(IDsTarget)
                        IDmain = [IDmain(1:ids-1),NaN,IDmain(ids:end)];
                        DetectedCubePose = [DetectedCubePose(1:ids-1),...
                            invalidPose,DetectedCubePose(ids:end)];
                    else
                        IDmain = [IDmain,NaN];
                        DetectedCubePose = [DetectedCubePose,invalidPose];
                    end
                end
            end
        end
        IDmainDetected = IDmain(~isnan(IDmain));
    else
        IDmainDetected = [];
    end
        
    % Check the detected Cube Poses

    if ~isempty(IDmainDetected)
        if isequal(sort(IDmainDetected),sort(IDsTarget))
            % 1st case
            if isempty(CubePoseBefore)
                clear CubePoseBefore
            end
            CubePose = DetectedCubePose;
%             for cubes = 1:size(CubePose,2)
%                 if size(CubePoseBefore,2) ~= 1
%                     if ~PoseSimilarValues(CubePose(cubes),CubePoseBefore(cubes),0.6)
%                         error("stop");
%                     end
%                 end
%             end
        elseif length(IDmainDetected) < length(IDsTarget)
            % 4th case
            warning('Less detected Cubes than expected');
            IDNotDetected = setdiff(IDsTarget,IDmainDetected);
            
            if ~isempty(CubePoseBefore)
                for oldtags = 1:length(IDsTarget)
                    IDsOld(oldtags) = CubePoseBefore(oldtags).ID;
                    if InBorder(CubePoseBefore(oldtags), ...
                            PoseSlaveRelToMaster,IntrinsicsMaster,...
                            IntrinsicsSlave,VideoObject)
                        IDBorder(oldtags) = CubePoseBefore(oldtags).ID;
                    end
                end
                if exist('IDBorder','var')
                    IDBorder = IDBorder(IDBorder ~= 0);
                else 
                    IDBorder = [];
                end
                for tags = 1:length(IDsTarget)
                    if ismember(CubePoseBefore(tags).ID, IDmainDetected)
                        CubePose(tags) = DetectedCubePose(IDmain==DetectedCubePose(tags).ID);
                    elseif ismember(CubePoseBefore(tags).ID, IDNotDetected) ...
                            && ~ismember(CubePoseBefore(tags).ID,IDBorder)
                        % 4b case
                        CubePose(tags) = CubePoseBefore(IDsOld==CubePoseBefore(tags).ID);
                    else
                        % 4a case
                        CubePose(tags) = AprilTagPose(zeros(4,4),NaN, tagSize);
                    end
                end
            else
                CubePose = DetectedCubePose;
            end
            
        else
            error('the given main target IDs may be incorrect');
        end
    else 
        % 2nd case
        CubePose =[];
    end
end