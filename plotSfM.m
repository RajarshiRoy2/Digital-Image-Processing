function plotSFM(A, S)
% A: Affine motion matrix
% S: Shape matrix

% Ploting 3D shape
figure, hold off, plot3(S(2, :), S(3, :), S(1, :), 'r.'), axis equal
saveas(gca, "modelResult.fig");

nc = size(A,1)/2; % Number of cameras

k = zeros(nc, 3); % Camera positions
for p = 1:nc
    k(p, :) = cross(A(p, :), A(p+nc, :));
    k(p, :) = k(p, :) ./ sqrt(sum(k(p, :).^2));
end

% Ploting 3D camera positions
figure, hold off, plot3(k(:, 2), k(:, 3), k(:, 1));
saveas(gca, "cameraPositions.jpg");
% Ploting camera motion, one for each dimension
figure, hold off, 
subplot(1,3,1), plot(k(:, 2));
subplot(1,3,2), plot(k(:, 3));
subplot(1,3,3), plot(k(:, 1));
saveas(gca, "cameraMotion.jpg");

zip('ModelingResults', {'cameraMotion.jpg', 'cameraPositions.jpg', 'keyPoints.jpg', 'modelResult.fig', 'trackedPointsFirst.jpg','trackedPointsFirst.jpg'});
end