classdef CalibratedVectorAprilTag < Vector

    properties
        ID double
        number double
        name char
    end

    methods
        function obj = CalibratedVectorAprilTag(Translation,AprilTagID,PointNumber,Name)
            obj@Vector(Translation);
            if nargin == 4
                obj.ID = AprilTagID;
                obj.number = PointNumber;
                obj.name = Name;
            elseif nargin == 3
                obj.ID = AprilTagID;
                obj.number = PointNumber;
            elseif nargin == 2
                obj.ID = AprilTagID;
            end
        end

        function bool = isVecInAprilTagPose(obj,AprilTagPose)
            if obj.ID == AprilTagPose.ID
                bool = true;
            else 
                bool = false;
            end
        end
    end
end