classdef AprilTagPose < Pose

    properties
        ID double
        CornersLoc double
        TagSize double
    end

    methods
        function obj = AprilTagPose(Transformation,AprilTagID,TagSize,CornersLocation)
            obj@Pose(Transformation);
            if nargin == 4
                obj.ID = AprilTagID;
                obj.CornersLoc = CornersLocation;
                obj.TagSize = TagSize;
            end
            if nargin == 3
                obj.ID = AprilTagID;
                obj.TagSize = TagSize;
            end
            if nargin == 2
                obj.ID = AprilTagID;
            end
        end
        function Image= DrawAprilTagPoseInImage(obj, Image, Intrinsics)
            if validAprilTagPose(obj)
                worldPoints = [0 0 0; obj.TagSize/2 0 0; 0 obj.TagSize/2 0; 0 0 obj.TagSize/2];
                imagePoints = worldToImage(Intrinsics,rigid3d(obj.T'),worldPoints);
                Image = insertShape(Image,"Line",[imagePoints(1,:) imagePoints(2,:); ...
                imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                "Color",["red","green","blue"],"LineWidth",7);
            end
        end
        function Image = DrawDetectedTagInImage(obj,Image)
            if validAprilTagPose(obj) && ~isempty(obj.CornersLoc)
                Image = insertShape(Image,"Polygon",obj.CornersLoc,...
                    "Color","red","LineWidth",3);
                Image = insertText(Image,obj.CornersLoc(1,:),obj.ID,"BoxOpacity",1,"FontSize",25);
            end
        end
        function bool = validAprilTagPose(obj)
            bool = ~isnan(obj.ID);
        end
    end
end