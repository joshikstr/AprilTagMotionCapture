
function varargout = readAprilTagModi(I, varargin)
%readAprilTag Read and estimate pose for AprilTags in images.
%   [id,loc] = readAprilTag(I) returns the tag IDs and locations in image
%   I. id is an N-element vector of tag IDs and loc is a 4-by-2-by-N matrix
%   of [x,y] locations of the 4 corners of the tag. I is a truecolor or
%   grayscale image.
%
%   [id, loc] = readAprilTag(I, tagFamily) restricts search to the tag
%   families specified in tagFamily. tagFamily is a specific family or a
%   string vector. Supported families are: "tag16h5", "tag25h9",
%   "tag36h11", "tagCircle21h7", "tagCircle49h12", "tagCustom48h12",
%   "tagStandard41h12", "tagStandard52h13". Use this syntax to improve
%   execution time when the tag family is known. Otherwise, the function
%   searches for "all" (default) supported tag families.
%
%   [id, loc, pose] = readAprilTag(..., intrinsics, tagSize) additionally
%   estimates the pose of the tags with respect to the camera. intrinsics
%   is a cameraIntrinsics object and tagSize is a scalar specifying the
%   length of the side of a tag in world units, for example in millimeters.
%   The tag size needs to be measured between the outer black edges of the
%   tags. pose is a N-element array of rigid3d objects that encapsulate 3-D
%   rigid transformations in the same world units as tagSize. The origin of
%   the tag frames are located at the center of the tags.
%
%   [..., detectedFamily] = readAprilTag(...) additionally returns an
%   N-element string vector containing the recognized tag families.
%
%   Notes
%   -----
%   - Consider using tag36h11 for applications that require real-time
%   performance while being robust to false positive detections. The
%   tagStandard41h12 family is the recommended choice if an acceptable
%   tradeoff between detection time and the number of supported IDs is
%   desired.
%
%   - Pre-generated tags for all the supported families can be found here:
%   https://github.com/AprilRobotics/apriltag-imgs
%
%   Example 1: Read AprilTags in image
%   ----------------------------------
%   % Read an image
%   I = imread("aprilTagsMulti.jpg");
%
%   % Tag families
%   tagFamily = ["tag36h11", "tagCircle21h7", "tagCircle49h12", ...
%       "tagCustom48h12", "tagStandard41h12"];
%
%   % Get the tag IDs, location and detectedFamilies
%   [id, loc, detectedFamily] = readAprilTag(I, tagFamily);
%
%   for idx = 1:length(id)
%       % Display the ID and tag family
%       disp("Detected Tag ID, Family: " + id(idx) + ", " ...
%           + detectedFamily{idx});
%
%       % Insert markers to indicate the locations
%       markerRadius = 8;
%       numCorners = size(loc,1);
%       I = insertShape(I, "FilledCircle", [loc(:,:,idx), repmat(markerRadius, ...
%           numCorners, 1)], "Color", "red", "Opacity", 1);
%   end
%
%   % Display image
%   imshow(I)
%
%   Example 2: Estimate AprilTag poses
%   ----------------------------------
%   % Read an image
%   I = imread("aprilTag36h11.jpg");
%
%   % Load camera intrinsics and specify measured tag size
%   data = load("camIntrinsicsAprilTag.mat");
%   intrinsics = data.intrinsics;  
%
%   % Size of the tags in meters
%   tagSize = 0.04;
%
%   % Undistort image
%   I = undistortImage(I, intrinsics, "OutputView", "same");
%
%   % Estimate tag poses
%   [id, loc, pose] = readAprilTag(I, "tag36h11", intrinsics, tagSize);
%
%   % Origin and Axes vectors for the tag frames
%   worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];
%
%   % Visualize the tag frames
%   for i = 1:length(pose)
%       % Get image coordinates for axes
%       imagePoints = worldToImage(intrinsics, pose(i).Rotation, ...
%                     pose(i).Translation, worldPoints);
%
%       % Draw colored axes
%       I = insertShape(I, "Line", [imagePoints(1,:) imagePoints(2,:); ...
%           imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
%           "Color", ["red", "green", "blue"], "LineWidth", 7);
%
%       I = insertText(I, loc(1,:,i), id(i), "BoxOpacity", 1, "FontSize", 25);
%   end
%
%   % Display image
%   imshow(I)
%
%   See also readBarcode, cameraIntrinsics, worldToImage, rigid3d,
%       insertShape, insertMarker, insertText

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen


narginchk(1,4);
[tagFamily, estimatePose, intrinsics, tagSize] = parseInputs(I, varargin{:});

% Convert to grayscale
if ~ismatrix(I)
    imgGray = rgb2gray(I);
else
    imgGray = I;
end


% Ensure image data is uint8
if ~isa(imgGray, 'uint8')
    imgUint8 = im2uint8(imgGray);
else
    imgUint8 = imgGray;
end

% Increasing this parameter will increase the speed of the detector at the
% cost of the detection distance.

quadDecimate = 1.0;
quadDecimate = checkQuadDecimate(quadDecimate);

if ~estimatePose
    nargoutchk(0,3);
    [varargout{1:nargout}] = readAprilTagID(imgUint8, tagFamily, quadDecimate);
else
    nargoutchk(0,4);
    [varargout{1:nargout}] = readAprilTagIDPose(imgUint8, tagFamily, intrinsics, tagSize, quadDecimate);
end

end

%--------------------------------------------------------------------------
% isSimMode - check if simulation mode or codegen mode
%--------------------------------------------------------------------------
function out = isSimMode()
    out = isempty(coder.target);
end

%--------------------------------------------------------------------------
% parseInputs - Parse the input arguments
%--------------------------------------------------------------------------
function [tagFamily, estimatePose, intrinsics, tagSize] = parseInputs(I, varargin)


vision.internal.inputValidation.validateImage(I);

if nargin == 1
    tagFamily = 'all';
    tagSize = 0;    
    
    % allocate memory for intrinsicsObj by assigning default values for camera
    % parameters
    intrinsics = cameraIntrinsics(ones(1,2),ones(1,2),ones(1,2));
end

estimatePose = nargin > 2;

if nargin == 2 % readAprilTag(I, tagFamily)
    
    tagFamily = varargin{1};
    tagSize = 0;
    % allocate memory for intrinsicsObj by assigning default values for camera
    % parameters
    intrinsics = cameraIntrinsics(ones(1,2),ones(1,2),ones(1,2));
    
elseif nargin == 3 % readAprilTag(I, intrinsics, tagSize)
    
    tagFamily = 'all';
    intrinsics = varargin{1};
    tagSize = varargin{2};
    
elseif nargin > 3 % readAprilTag(I, tagFamily, intrinsics, tagSize)
    
    tagFamily  = varargin{1};
    intrinsics = varargin{2};
    tagSize = varargin{3};
end

% validate tag Family
tagFamily = checkTagFamily(tagFamily);

if estimatePose
    vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    intrinsics, true, mfilename, 'intrinsics'); 
    % validate tag Size
    tagSize = checkTagSize(tagSize);
end
end

%--------------------------------------------------------------------------
% readAprilTagID - Call AprilTag tag ID decoding
%--------------------------------------------------------------------------
function [id, loc, detectedFamily] = readAprilTagID(Img, tagFamily, quadDecimate)


if isSimMode()
    [id, loc, detectedFamily] = vision.internal.aprilTagReader(Img.', tagFamily, false, quadDecimate);
else
    [id, loc, detectedFamily] = vision.internal.buildable.readAprilTagBuildable.readAprilTagID(Img.', ...
        tagFamily, false, quadDecimate);
end

loc = loc + 1; % AprilTag uses zero-indexing for images

if isSimMode()
    %output string array for detected families
    detectedFamily = string(detectedFamily);
end

end

%--------------------------------------------------------------------------
% readAprilTagIDPose - Call AprilTag tag ID decoding and pose estimation
%--------------------------------------------------------------------------
function [id, loc, pose, detectedFamily] = readAprilTagIDPose(I, tagFamily, intrinsics, tagSize, quadDecimate)

focalLength = intrinsics.FocalLength;
principalPoint = intrinsics.PrincipalPoint;

if isSimMode()
    [id, loc, detectedFamily, rotMatrices, transVectors] = vision.internal.aprilTagReader(I.', tagFamily, ...
        true, quadDecimate, focalLength, principalPoint, tagSize);
else
    [id, loc, detectedFamily, rotMatrices, transVectors] = vision.internal.buildable.readAprilTagBuildable.readAprilTagID(I.', tagFamily, ...
        true, quadDecimate, focalLength, principalPoint, tagSize);
end

count = 1;
invalidIdx = [];

if isSimMode()
    pose = rigid3d.empty;
    for idx = 1:size(rotMatrices, 3)

        % Only accept tags with valid poses. An invalid rotation matrix points
        % to an incorrect detection. This is more prominent in the smaller tag
        % families like tag16h5 and tag25h9.
        try
            vision.internal.inputValidation.validateRotationMatrix(...
                rotMatrices(:,:,idx), 'readAprilTag', 'rotationMatrix');
            pose(count) = rigid3d(rotMatrices(:,:,idx), transVectors(:,:,idx)');
            count = count + 1;
        catch
            invalidIdx = [invalidIdx idx]; %#ok<AGROW>
        end
    end
    
else
    
    % allocating memory for rigid3d object array
    dataType = 'double';
    T = eye(4, 4, dataType);
    
    % Create a dummy object
    pose = rigid3d(T);
    
    countValid = 1;
    for idx = 1:size(rotMatrices, 3)
        if checkRotationMatrix(rotMatrices(:,:,idx))
            
            %creating rigid3d object array
            pose(countValid) = rigid3d(rotMatrices(:,:,idx),transVectors(:,:,idx)');
            countValid = countValid+1;
        else
            invalidIdx = [invalidIdx idx];
        end
    end
    
end
id(invalidIdx) = [];
loc(:,:,invalidIdx) = [];
detectedFamily(invalidIdx) = [];

loc = loc + 1; % AprilTag uses zero-indexing for images

if isSimMode()
    detectedFamily = string(detectedFamily);
end

end

%--------------------------------------------------------------------------
% checkTagFamily - Validate AprilTag families
%--------------------------------------------------------------------------
function tagFamily = checkTagFamily(tagFamily)
validateattributes(tagFamily, {'char', 'string', 'cell'}, {'nonempty', 'vector'}, 'readAprilTag', 'tagFamily');

supportedFamilies = vision.internal.supportedAprilTagFamilies();

if ischar(tagFamily) || isStringScalar(tagFamily)
    
    if ~isSimMode()
        n = size(supportedFamilies,2);
        validFamiliesChar = cell(1,n+1);
        validFamiliesChar{1} = 'all';
        for i = 1:n
            validFamiliesChar{i+1} = supportedFamilies{i};
        end
        
        % a tag family is valid if it matches with families present in
        % validFamilies
        isValidFamily = any(strcmp(validFamiliesChar,tagFamily));
        
        formatMsg = strjoin(supportedFamilies, ', ');
        coder.internal.errorIf(~isValidFamily,'vision:apriltag:unrecognizedStringChoice', formatMsg);
        
    else 
        validFamiliesChar = [{'all'}, supportedFamilies(:)'];
        tagFamily = validatestring(tagFamily, validFamiliesChar, 'readAprilTag', 'tagFamily');
    end
else
        
    coder.internal.errorIf(~iscellstr(tagFamily) && ...
    ~isstring(tagFamily),'vision:apriltag:invalidStringList');
    
    if isSimMode()
        isValidFamily = all(ismember(tagFamily, supportedFamilies));
    else
        count = 0;
        if iscellstr(tagFamily)         
            for i = 1:size(tagFamily,2)
                count = count+any(strcmp(supportedFamilies,tagFamily{i}));
            end
            isValidFamily = (count == size(tagFamily,2));
        else
            isValidFamily = any(strcmp(supportedFamilies, tagFamily));
        end
    end
    
    formatMsg = strjoin(supportedFamilies, ', ');
    coder.internal.errorIf(~isValidFamily,'vision:apriltag:unrecognizedStringChoice', formatMsg);

    if isSimMode()
        tagFamily = cellstr(tagFamily);
    end
end

end

%--------------------------------------------------------------------------
% checkTagSize - Validate tag size
%--------------------------------------------------------------------------
function tagSize = checkTagSize(tagSize)
validateattributes(tagSize, {'numeric'}, {'finite', 'real', 'nonsparse', ...
    'scalar', 'positive'}, 'readAprilTag');
end

%--------------------------------------------------------------------------
% checkQuadDecimate - Validate quadDecimate parameter
%--------------------------------------------------------------------------
function quadDecimate = checkQuadDecimate(quadDecimate)
validateattributes(quadDecimate, {'numeric'}, {'finite', 'real', 'nonsparse', ...
    'scalar', 'positive', '>=', 1}, 'readAprilTag');
end

%--------------------------------------------------------------------------
% checkRotationMatrix - Validate rotation matrix
%--------------------------------------------------------------------------
function validFlag = checkRotationMatrix(M)

validateattributes(M, {'numeric'}, ...
    {'finite', '2d', 'real', 'nonsparse', 'size', [3,3]});

if abs(det(double(M))-1) > 1e-3
    validFlag = false;
else
    M = double(M);
    MM = M*M';
    I = eye(3);
    if max(abs(MM(:)-I(:))) > 1e-3
        validFlag = false;
    end
    validFlag = true;
end
end