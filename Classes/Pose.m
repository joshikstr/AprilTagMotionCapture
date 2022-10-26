classdef Pose

    properties
        Trans double
        Rot double
        T double
    end

    methods
        function obj = Pose(Transformation)
            obj.T = Transformation;
            obj.Rot = Transformation(1:3,1:3);
            obj.Trans = Transformation(1:3,4);
        end

        function obj = InversePose(obj)
            obj.T = inv(obj.T);
        end

        function Output = MultiplyPoses(Pose1,Pose2)
            Transformation = Pose1.T * Pose2.T;
            Output = Pose(Transformation);
        end
        
        function Output = PoseRotToQuat(obj)
            Output = quaternion(rotm2quat(obj.Rot));
        end

        function boolOut = PoseSimilarValues(obj,Pose,maxDiffValue)
            diff = obj.T - Pose.T;
            boolOut = ~any(abs(diff(:))>maxDiffValue);
        end
    end
end