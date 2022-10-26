classdef AnatomicalVectorCam < Vector

    properties
        number double
        name char
    end

    methods
        function obj = AnatomicalVectorCam(Translation,PointNumber,Name)
            obj@Vector(Translation);
            if nargin == 3
                obj.number = PointNumber;
                obj.name = Name;
            elseif nargin == 2
                obj.number = PointNumber;
            end
        end
        function Image = DrawVectorInImage(obj,Pose,Image,Intrinsics)
            markerRadius = 12;
            [uVec,vVec] = VecCamToPixel(obj,Intrinsics);
            [uPose,vPose] = VecCamToPixel(Vector(Pose.Trans),Intrinsics);
            Image = insertShape(Image, "Line", [uVec,vVec,uPose,vPose],"Color","cyan");
            Image = insertShape(Image, 'FilledCircle', [uVec,vVec, markerRadius], "Color", "cyan");
        end
    end
end