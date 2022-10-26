function bool = InBorder(Pose,PoseSlaveRelToMaster,IntrinsicsMaster, IntrinsicsSlave,VideoObject)
    % InBorder ckecks if the xyValues AprilTag Middle are inside the
    % border range (10% of the image edges)
    %
    % by Joshua Köster
    %
    
    transVec = Vector(Pose.Trans);

    % check Master
    [uMaster,vMaster] = VecCamToPixel(transVec,IntrinsicsMaster);
    xMin = 0.1*VideoObject.Width;
    xMax = 0.9*VideoObject.Width;
    yMin = 0.1*VideoObject.Height;
    yMax = 0.9*VideoObject.Height;
    if uMaster > xMin && uMaster < xMax ...
            && vMaster > yMin && vMaster < yMax
        bool = false;
    else 
        bool = true;
    end
    % Note that in the case that the Tag was detected in Relation to Slave 
    % the Tag has Value out of range in Master Video of VideoObject, thats
    % why we have to check again slave, if he wasnt in master

    if bool % check Slave (maybe the Tag was detected in Slave Cam before)
        PoseInSlave = MultiplyPoses(InversePose(PoseSlaveRelToMaster),Pose);
        transVec = Vector(PoseInSlave.Trans);
        [uSlave,vSlave] = VecCamToPixel(transVec,IntrinsicsSlave);
        if uSlave > xMin && uSlave < xMax ...
            && vSlave > yMin && vSlave < yMax
            bool = false;
        else 
            bool = true;
        end
    end
end