classdef CalibratedAprilTagGroup

    properties
        mainID double
        IDs double
        ID_positions struct
        CubeNumber double
        CubeName char
        PosesToMainID AprilTagPose
    end

    methods
        function obj = CalibratedAprilTagGroup(mainID,IDsGroup,position_of_IDs,CubeNumber,CubeName,PosesToMainID)
            if nargin == 6
                obj.mainID = mainID;
                obj.IDs = IDsGroup;
                obj.ID_positions = position_of_IDs;
                obj.CubeNumber = CubeNumber;
                obj.CubeName = CubeName;
                obj.PosesToMainID = PosesToMainID;
            end
            if nargin == 5 
                obj.mainID = mainID;
                obj.IDs = IDsGroup;
                obj.ID_positions = position_of_IDs;
                obj.CubeNumber = CubeNumber;
                obj.CubeName = CubeName;
            end
            if nargin == 4
                obj.mainID = mainID;
                obj.IDs = IDsGroup;
                obj.ID_positions = position_of_IDs;
                obj.CubeNumber = CubeNumber;
            end
            if nargin == 3
                obj.mainID = mainID;
                obj.IDs = IDsGroup;
                obj.ID_positions = position_of_IDs;
            end
        end
    end
end