close all
clear all
clc

f = 12;
s = 5 * 1.0e-3;
W = [1280, 1024];
Cx = W(1) / 2;
Cy = W(2) / 2;
K = [f / s, 0, Cx; 0, f / s, Cy; 0, 0, 1];
alph = atan2(500, 1000);
ca = cos(alph);
sa = sin(alph);

H = K * [ca, sa, 0; 0, 0, 0; -sa, ca, sqrt(1000^2 + 500^2)];
det(H)
H = H / norm(H(:));


%%
% Rotate the laser around the Xw of 45º
c45 = cosd(45);
s45 = sind(45);

H45 = K * [ca, sa * c45, 0; -s45, 0, 0; -sa, ca * c45, sqrt(1000^2 + 500^2)];
H45 = H45 / norm(H45(:));
det(H45)

%%
clc
H45T = H45';

% Solution 4 points having linear dependencies 
Pw = [200, 150, 100, 50;...
  -36, -49, -62, -75];
[h, h1, Pi, L] = calculateHomoProjecting(Pw, H45);
[h, h1, H45T(:), [L(1:4); 0 * L(5:end)]]

% Solution 4 points no linear dependencies
Pw = [200, 150, 100, 50;...
      -23, -49, -62, -62];
[h, h1, Pi, L] = calculateHomoProjecting(Pw, H45);
[h, h1, H45T(:), [L(1:4); 0 * L(5:end)]]

% Solution 8 points
Pw = [200, 200, 150, 150, 100, 100, 50, 50;...
  -23, -36, -36, -49, -49, -62, -62, -75];

[h, h1, Pi, L] = calculateHomoProjecting(Pw, H45);
[h, h1, H45T(:), [L(1:4); 0 * L(5:end)]]

%%
% simulation for error with added noises
clc
% Define first the values of the points for homographies given the indexes
Pw = [200, 200, 150, 150, 100, 100,  50,  50;...
      -23, -36, -36, -49, -49, -62, -62, -75];
nPointsExp = [4, 8];
indexes = [1, 4, 6, 7, 0, 0, 0, 0; ...
           1, 2, 3, 4, 5, 6, 7, 8];
         
% Variances of added normal noises
% The noise is the error in the measurement of the points
sigmaPw = [0, 0.001, 0.005, 0.01, 0.05, 0.1];
sigmaPixel = [0, 0.01, 0.05, 0.1];

% Definition of the theoretical points for error calculation
count = 1;
nbPointsPth = 22;
Pwth = ones(3, nbPointsPth);

for i = 1:size(Pw, 2) - 1
  p1 = Pw(:, i);
  p2 = Pw(:, i + 1);
  deltaP = (p2 - p1) / 3.0; % Divide the interval in 3 parts
  
  if (i == 7)
    numberParts = 4;
  else
    numberParts = 3;
  end
  
  for j = 1:numberParts 
    Pwth(1:2, count) = p1 + (j - 1) * deltaP;
    count = count + 1;
  end
end

% Repeat 100 times the experiments
for nStatist = 1:100
  for nExp = 1:2  
    nPoints = nPointsExp(nExp); % 4 or 8 points
    Pwi = ones(3, nPoints);     % laser points with added noise
    PwiNoNoise = ones(3, nPoints); % original laser points
    Pi = ones(3, nPoints);
    
    for i = 1:length(sigmaPw) % Sigmas of points in the world
      
      for j = 1:nPoints % Add noise to the Pw
        % adding laser noise to the points
        PwiNoNoise(1:2, j) = Pw(:, indexes(nExp, j));
        Pwi(1:2, j) = PwiNoNoise(1:2, j) + sigmaPw(i) * randn(2, 1);
      end
      
      for j = 1:length(sigmaPixel)
        % Project into the image and add image noise
        %Pi = H45 * Pwi;
        PiNoNoise = H45 * Pwi;
        PiNoNoise = PiNoNoise ./ PiNoNoise(3, :);
        
        for k = 1:nPoints
          % adding image noise to the observed points
          %Pi(1:2, k) = Pi(1:2, k) + sigmaPixel(j) * randn(2, 1);
          Pi(1:2, k) = PiNoNoise(1:2, k) + sigmaPixel(j) * randn(2, 1);
        end
        
        % Calculate the homography and the error with the theoretical one
        [h, h1, L] = calculateHomo(Pi, PwiNoNoise);
        
        % h1 can be just the opposite direction
        ErrH(nStatist, nExp, i, j) = min(norm(h1 - H45T(:)), norm(h1 + H45T(:)));
        % the inverse of the homography given by h1        
        h1 = reshape(h1, 3, 3)';  %Transpose is needed
        h1i = inv(h1);
        % Calculate the errors of the original points and the obtained by
        % h1
        SumMAEPwi = 0;
        
        Pith = H45 * Pwth;
        
        PwReal = h1i * Pith;
        
        for k = 1:nbPointsPth
          % alculate ErrPwk, i.e. the back projection of the points and error
          % ErrPwk = Pwth(1:2, j) - ErrH;
          ErrPwk = abs(Pwth(1:2, k)) - abs(PwReal(1:2, k)); % You have to calculate this value
          SumMAEPwi = SumMAEPwi + norm(ErrPwk);
        end
        
        % Mean Absolute Error
        ErrMAEPw(nStatist, nExp, i, j) = SumMAEPwi / nPoints;
      end      
    end
  end
end

% Calculate the mean of the 100 experimental errors
for nExp = 1:2
  for i = 1:length(sigmaPw)
    for j = 1:length(sigmaPixel)
      AverErrH(nExp, i, j) = mean(ErrH(:, nExp, i, j));
      AverMAEPw(nExp, i, j) = mean(ErrMAEPw(:, nExp, i, j));
    end
  end
end

% Write the values of AverErrH in Excel
fileNameErrH = 'ErrH.xlsx';
fileNameMAEPw = 'ErrMAEPw.xlsx';
TextoSheet = {'4 Points Homography', '8 Points Homography'};
for nExp = 1:2
  for i = 1:length(sigmaPw)
    s = strcat('sigmaPw =', {' '}, num2str(sigmaPw(i)));
    writematrix(s{1} , fileNameErrH, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65 + i), int2str(1)));
               
    writematrix(s{1} , fileNameMAEPw, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65 + i), int2str(1)));
               
    for j = 1:length(sigmaPixel)
      
      if (i == 1)
        s = strcat('sigmaIm =', {' '}, num2str(sigmaPixel(j)));
        writematrix(s{1}, ...
                 fileNameErrH, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65), int2str(j + 1)));
        writematrix(s{1} , fileNameMAEPw, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65), int2str(j + 1)));
      end
      
      writematrix(AverErrH(nExp, i, j), fileNameErrH, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65 + i), int2str(j + 1)));
      writematrix(AverMAEPw(nExp, i, j), fileNameMAEPw, 'Sheet', ...
                 TextoSheet{nExp}, ...
                 'Range', strcat(char(65 + i), int2str(j + 1)));
    end
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [h, h1, Pi, L] = calculateHomoProjecting(Pw, H)
nPoints = size(Pw, 2);
Pw = [Pw; ones(1, nPoints)];

Pi = H * Pw;
for i = 1:nPoints
  Pi(:, i) = Pi(:, i) / Pi(3, i);
end

% Calculate the H by using the points
[h, h1, L] = calculateHomo(Pi, Pw);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [h, h1, L] = calculateHomo(Pi, Pw)
nPoints = size(Pi, 2);
A = zeros(nPoints*2, 9);

for i = 1:nPoints
  A(i*2 - 1, 1) = -Pw(1, i);
  A(i*2 - 1, 2) = -Pw(2, i);
  A(i*2 - 1, 3) = -1;
  A(i*2 - 1, 7) = Pw(1, i) * Pi(1, i);
  A(i*2 - 1, 8) = Pw(2,i) * Pi(1,i);
  A(i*2 - 1, 9) = Pi(1,i);
  
  A(i*2, 4) = -Pw(1, i);
  A(i*2, 5) = -Pw(2, i);
  A(i*2, 6) = -1;
  A(i*2, 7) = Pw(1, i) * Pi(2, i);
  A(i*2, 8) = Pw(2, i) * Pi(2, i);
  A(i*2, 9) = Pi(2, i);
end

% Solution with eigenvalues
[V, ~] = eig(A'* A);
L = eig(A' * A);
h = V(:, 1);

% Solution with singular values
[~, ~, V1] = svd(A);
h1 = V1(:, end);

return;
end
