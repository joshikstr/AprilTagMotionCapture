classdef Vector

    properties
        vec double
    end

    methods
        function obj = Vector(Translation)
            obj.vec = Translation;
        end

        function obj = CoordinateTransformation(obj,Pose)
            hVec = Pose.T * e2h(obj.vec);
            obj.vec = h2e(hVec);
        end

        function Image = DrawPointInImage(obj, Image, Intrinsics)
            markerRadius = 12;
            [u,v] = VecCamToPixel(obj,Intrinsics);
            Image = insertShape(Image, "FilledCircle", [u,v,markerRadius],"Color","cyan");
        end

        function boolOut = VecSimilarValues(obj,Vector)
            obj.vec = round(obj.vec,3);
            Vector.vec = round(Vector.vec,3);
            if obj.vec==Vector.vec
                boolOut = true;
            else
                boolOut = false;
            end
        end

        function [u,v] = VecCamToPixel(obj,Intrinsics)
            u = Intrinsics.FocalLength(1,1) * (obj.vec(1,1)/obj.vec(3,1))+Intrinsics.PrincipalPoint(1,1); 
            v = Intrinsics.FocalLength(1,2) * (obj.vec(2,1)/obj.vec(3,1))+Intrinsics.PrincipalPoint(1,2); 
        end
    end
end