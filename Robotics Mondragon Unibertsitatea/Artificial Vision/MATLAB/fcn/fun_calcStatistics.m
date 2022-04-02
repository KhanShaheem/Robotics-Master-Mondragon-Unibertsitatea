function [accuracy, repeatability] = fun_calcStatistics (W1,W2,W3,L1,L2,D1)

load('real_meas');

%% Circles
% Accuracy
relative_error = abs(D1_real - mean(D1))/D1_real * 100;
accuracy.D1 = 100 - relative_error;

% Repeatability
std_error = std(D1)/D1_real * 100;
repeatability.D1 = 100 - std_error;

%% Lines
% Accuracy (W1)
relative_error = abs(W1_real - mean(W1))/W1_real * 100;
accuracy.W1 = 100 - relative_error;
% Repeatability (W1)
std_error = std(W1)/W1_real * 100;
repeatability.W1 = 100 - std_error;

% Accuracy (W2)
relative_error = abs(W2_real - mean(W2))/W2_real * 100;
accuracy.W2 = 100 - relative_error;
% Repeatability (W2)
std_error = std(W2)/W2_real * 100;
repeatability.W2 = 100 - std_error;

% Accuracy (W3)
relative_error = abs(W3_real - mean(W3))/W3_real * 100;
accuracy.W3 = 100 - relative_error;
% Repeatability (W3)
std_error = std(W3)/W3_real * 100;
repeatability.W3 = 100 - std_error;

% Accuracy (L1)
relative_error = abs(L1_real - mean(L1))/L1_real * 100;
accuracy.L1 = 100 - relative_error;
% Repeatability (L1)
std_error = std(L1)/L1_real * 100;
repeatability.L1 = 100 - std_error;

% Accuracy (L2)
relative_error = abs(L2_real - mean(L2))/L2_real * 100;
accuracy.L2 = 100 - relative_error;
% Repeatability (L2)
std_error = std(L1)/L2_real * 100;
repeatability.L2 = 100 - std_error;
