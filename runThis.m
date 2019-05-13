
input = name;


v = VideoReader(input);
% Creating directory to store images after processing
currentDir = 'video/';
mkdir(currentDir);

count = 0;
count1 = 0;
        while hasFrame(v)
             image = readFrame(v);%reading everyother frame  
            if mod(count,2)==0
              
               F = rgb2gray(image);%filter

               % Store frame into directory
               file = [sprintf('frame%d',count1) '.jpg'];
               fullname = fullfile(currentDir,'Frames',file);
               imwrite(F,fullname); % Use this to get a video with edge detected frames
               count1 = count1 + 1;
            end
            count = count + 1;
        end


folder = 'video/Frames';
im = readImages(folder, 0:count1-1);

tau = 0.06;                                 % Threshold for harris corner detection
[pt_y, pt_x] = getKeypoints(im{1}, tau);    % Prob 1.1: keypoint detection
ws = 15;                                    % Tracking ws x ws patches
[track_x, track_y] = ...                    % Keypoint tracking
    trackPoints(pt_x, pt_y, im, ws);
  
%load './tracks.mat';


figure, imagesc(im{1}), hold off, axis image, colormap gray
hold on, plot(track_x', track_y', 'r')
saveas(gca, "trackedPointsFirst.jpg");
figure, imagesc(im{end}), hold off, axis image, colormap gray
hold on, plot(track_x', track_y', 'r')
saveas(gca, "trackedPointsLast.jpg");
%pause;

valid = ~any(isnan(track_x), 2) & ~any(isnan(track_y), 2); 

[A, S] = affineSFM(track_x(valid, :), track_y(valid, :));

plotSfM(A, S);

function [A, S] = affineSFM(x, y)
% Function: Affine structure from motion algorithm

%% Normalize x, y to zero mean
xn = x - mean(x(:));
yn = y - mean(y(:));

%% Create measurement matrix
D = [xn' ; yn'];

%% Decompose and enforce rank 3
[U, W, V] = svd(D);
U3 = U(:, 1:3);
A = U(:, 1:3) * sqrt(W(1:3, 1:3));   % 102x3
S = sqrt(W(1:3, 1:3)) * V(:, 1:3)';  % 3x400

%% Apply orthographic constraints

k = repmat([1; 1; 0], 51, 1);
abc = zeros(153, 3);
def = zeros(3, 153);
j = 1;
for i=1:51
   % Grabs the a, b, and c variables for the point
   abc(j, :) = A(i, :);
   abc(j+1, :) = A(i+51, :);
   abc(j+2, :) = A(i, :);
   
   % Grabs the d, e, and f variables for the point
   def(:, j) = A(i, :)';
   def(:, j+1) = A(i+51, :)';
   def(:, j+2) = A(i+51, :)';
   j = j + 3;
end

X=[];
totalRows = size(abc(:,1));


check = abc(1, :)'.*def(:, 1)';
for i=1:totalRows(1,1)
    X(i, :) = reshape(abc(i, :)'*def(:, i).', [1, 9]);
end

L = reshape(k \ X, [3, 3]);
C = chol(L)';    % Computes Cholesky factorization
A = A * C';

S = C^(-1) * S;
S = S.*200;     % Reshapes S according to given assignment example
S(1,:) = S(1,:)*1;
S(2,:) = S(2,:)*(-0.9);
S(3,:) = S(3,:) + 100;
end
function im = readImages(folder, nums)
im = cell(numel(nums),1);
t = 0;
for k = nums,
    t = t+1;
    im{t} = imread(fullfile(folder, ['frame' num2str(k) '.jpg']));
    im{t} = im2single(im{t});
end
end

function [track_x, track_y] = trackPoints(pt_x, pt_y, im, ws)
% Tracking initial points (pt_x, pt_y) across the image sequence
% track_x: [Number of keypoints] x [2]
% track_y: [Number of keypoints] x [2]
 
% Initialization
N = numel(pt_x);
nim = numel(im);
track_x = zeros(N, nim);
track_y = zeros(N, nim);
track_x(:, 1) = pt_x(:);
track_y(:, 1) = pt_y(:);
 
for t = 1:nim-1
    disp(t)
    [track_x(:, t+1), track_y(:, t+1)] = ...
            getNextPoints(track_x(:, t), track_y(:, t), im{t}, im{t+1}, ws);
end
 
 end
function [x2, y2] = getNextPoints(x, y, im1, im2, ws)
% Iterative Lucas-Kanade feature tracking
% x,  y : initialized keypoint position in im2
% x2, y2: tracked keypoint positions in im2
% ws: patch window size
%% 1. Compute gradients from Im1 (get Ix and Iy) 
    
fil = fspecial('Gaussian',11,1); 
im1 = imfilter(im1, fil);           % Smooths images
im2 = imfilter(im2, fil);
[imH ,imW] = size(im1);

[Ix, Iy] = gradient(im1);

hw = floor(ws/2);
[X, Y] = meshgrid(-hw:hw,-hw:hw);   % Offset patch
numIter = 5;
 
for p=1:size(x)   
    x2 = x;   % Initializes (x',y') = (x,y)
    y2 = y;
    
    %% 2. Grab patches of Ix, Iy, and im1. 
    %    Hint 1: use “[X, Y] = meshgrid(-hw:hw,-hw:hw);” to get patch index, where hw = floor(ws/2);
    %    Hint 2: use “interp2” to sample non-integer positions.
    x1 = x(p) + X;
    y1 = y(p) + Y;
    [Less_boundy] = find(y1 <= 1);[Less_boundx] = find(x1 <= 1);
    [Great_boundy] = find(y1 >= imW);[Great_boundx] = find(x1 >= imH);
    check = size(Less_boundy) + size(Less_boundx) + size(Great_boundy) + size(Great_boundx);
    if (check(1,1) > 0)
        continue;           % Checks if patch is good
    end
    
    patch1 = interp2(im1, x1, y1);
    patchIx = interp2(Ix, x1, y1);
    patchIy = interp2(Iy, x1, y1);
    
    %% - Set up matrix A
    Ix2 = sum(patchIx.^(2));
    Ixy = sum(patchIx.*patchIy);
    Iy2 = sum(patchIy.^(2));
    
    A = [ sum(Ix2) sum(Ixy); sum(Ixy) sum(Iy2)];
   
    for iter = 1:numIter                           % 5 iterations should be sufficient
        %% Check if tracked patch are outside the image. Only track valid patches. 

        % For each keypoint 
        %% - grab patch2 (centered at x2,y2) 
        patch2x = x2(p) + X;
        patch2y = y2(p) + Y;
        [Less_boundy] = find(patch2y <= 1);[Less_boundx] = find(patch2x <= 1);
        [Great_boundy] = find(patch2y >= imW);[Great_boundx] = find(patch2x >= imH);
        check = size(Less_boundy) + size(Less_boundx) + size(Great_boundy) + size(Great_boundx);
        if (check(1,1) == 0)    %checks if patch is valid
            patch2 = interp2(im2, patch2x, patch2y);

            %% - compute It = patch2 – patch1 
            It = (patch2 - patch1);

            %% - Set vector b 
            Ixt = sum(patchIx.*It);
            Iyt = sum(patchIy.*It);
            b = -[ sum(Ixt); sum(Iyt)];

            %% - Solve linear system d = A\b. 
            d = A\b;

            %% - x2(p)=x2(p)+d(1); y2(p)=y2(p)+d(2);   -> Update the increment end
            x2(p) = x2(p) + d(1);
            y2(p) = y2(p) + d(2);
        end
    end
    x = x2;
    y = y2;
end
end
