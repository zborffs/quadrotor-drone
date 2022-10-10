clear; clc;

addpath('utils')

%% Data generated from Random sampling of Normal Distribution
raw_data = [
    -1.276 -1.218 -0.453 -0.350  0.723  0.676 -1.099 -0.314 -0.394 -0.633
    -0.318 -0.799 -1.664  1.391  0.382  0.733  0.653  0.219 -0.681  1.129
    -1.377 -1.257  0.495 -0.139 -0.854  0.428 -1.322 -0.315 -0.732 -1.348
     2.334 -0.337 -1.955 -0.636 -1.318 -0.433  0.545  0.428 -0.297  0.276
    -1.136  0.642  3.436 -1.667  0.847 -1.173 -0.355  0.035  0.359  0.930
     0.414 -0.011  0.666 -1.132 -0.410 -1.077  0.734  1.484 -0.340  0.789
    -0.494  0.364 -1.237 -0.044 -0.111 -0.210  0.931  0.616 -0.377 -0.433
     1.048  0.037  0.759  0.609 -2.043 -0.290  0.404 -0.543  0.486  0.869
     0.347  2.816 -0.464 -0.632 -1.614  0.372 -0.074 -0.916  1.314 -0.038
     0.637  0.563 -0.107  0.131 -1.808 -1.126  0.379  0.610 -0.364 -2.626
     2.176  0.393 -0.924  1.911 -1.040 -1.168  0.485  0.076 -0.769  1.607
    -1.185 -0.944 -1.604  0.185 -0.258 -0.300 -0.591 -0.545  0.018 -0.485
     0.972  1.710  2.682  2.813 -1.531 -0.490  2.071  1.444 -1.092  0.478
     1.210  0.294 -0.248  0.719  1.103  1.090  0.212 -1.185 -0.338 -1.134
     2.647  0.777  0.450  2.247  1.151 -1.676  0.384  1.133  1.393  0.814
     0.398  0.318 -0.928  2.416 -0.936  1.036  0.024 -0.560  0.203 -0.871
     0.846 -0.699 -0.368  0.344 -0.926 -0.797 -1.404 -1.472 -0.118  1.456
     0.654 -0.955  2.907  1.688  0.752 -0.434  0.746  0.149 -0.170 -0.479
     0.522  0.231 -0.619 -0.265  0.419  0.558 -0.549  0.192 -0.334  1.373
    -1.288 -0.539 -0.824  0.244 -1.070  0.010  0.482 -0.469 -0.090  1.171
     1.372  1.769 -1.057  1.646  0.481 -0.600 -0.592  0.610 -0.096 -1.375
     0.854 -0.535  1.607  0.428 -0.615  0.331 -0.336 -1.152  0.533 -0.833
    -0.148 -1.144  0.913  0.684  1.043  0.554 -0.051 -0.944 -0.440 -0.212
    -1.148 -1.056  0.635 -0.328 -1.221  0.118 -2.045 -1.977 -1.133  0.338
     0.348  0.970 -0.017  1.217 -0.974 -1.291 -0.399 -1.209 -0.248  0.480
     0.284  0.458  1.307 -1.625 -0.629 -0.504 -0.056 -0.131  0.048  1.879
    -1.016  0.360 -0.119  2.331  1.672 -1.053  0.840 -0.246  0.237 -1.312
     1.603 -0.952 -0.566  1.600  0.465  1.951  0.110  0.251  0.116 -0.957
    -0.190  1.479 -0.986  1.249  1.934  0.070 -1.358 -1.246 -0.959 -1.297
    -0.722  0.925  0.783 -0.402  0.619  1.826  1.272 -0.945  0.494  0.050
    -1.696  1.879  0.063  0.132  0.682  0.544 -0.417 -0.666 -0.104 -0.253
    -2.543 -1.333  1.987  0.668  0.360  1.927  1.183  1.211  1.765  0.35
    -0.359  0.193 -1.023 -0.222 -0.616 -0.060 -1.319  0.785 -0.430 -0.298
     0.248 -0.088 -1.379  0.295 -0.115 -0.621 -0.618  0.209  0.979  0.906
    -0.099 -1.376  1.047 -0.872 -2.200 -1.384  1.425 -0.812  0.748 -1.093
    -0.463 -1.281 -2.514  0.675  1.145  1.083 -0.667 -0.223 -1.592 -1.278
     0.503  1.434  0.290  0.397 -0.837 -0.973 -0.120 -1.594 -0.996 -1.244
    -0.857 -0.371 -0.216  0.148 -2.106 -1.453  0.686 -0.075 -0.243 -0.170
    -0.122  1.107 -1.039 -0.636 -0.860 -0.895 -1.458 -0.539 -0.159 -0.420
     1.632  0.586 -0.468 -0.386 -0.354  0.203 -1.234  2.381 -0.388 -0.063
     2.072 -1.445 -0.680  0.224 -0.120  1.753 -0.571  1.223 -0.126  0.034
    -0.435 -0.375 -0.985 -0.585 -0.203 -0.556  0.024  0.126  1.250 -0.615
     0.876 -1.227 -2.647 -0.745  1.797 -1.231  0.547 -0.634 -0.836 -0.719
     0.833  1.289 -0.022 -0.431  0.582  0.766 -0.574 -1.153  0.520 -1.018
    -0.891  0.332 -0.453 -1.127  2.085 -0.722 -1.508  0.489 -0.496 -0.025
     0.644 -0.233 -0.153  1.098  0.757 -0.039 -0.460  0.393  2.012  1.356
     0.105 -0.171 -0.110 -1.145  0.878 -0.909 -0.328  1.021 -1.613  1.560
    -1.192  1.770 -0.003  0.369  0.052  0.647  1.029  1.526  0.237 -1.328
    -0.042  0.553  0.770  0.324 -0.489 -0.367  0.378  0.601 -1.996 -0.738
     0.498  1.072  1.567  0.302  1.157 -0.720  1.403  0.698 -0.370 -0.551
];

%% Preprocess data by concatenating all columns into single column
data = [];
for i = 1:size(raw_data, 1)
    data = [data; raw_data(i,:)'];
end


%% Compute summary statistics
fprintf("1. Summary Statistics\n")
summary_stats = compute_summary_stats(data);
print_summary_stats(summary_stats);

%% Location: does the data drift over time?
% One of the assumptions, which many of the hypothesis tests we would like 
% to run over this dataset, is: the dataset does not exhibit drift w.r.t.
% time. In other words, the mean value of the Gaussian random process
% doesn't itself vary w.r.t. time. Before applying any test that makes such
% an assumption, it is necessary to test the validity of the assumption so
% that the test's premises are true, i.e., so that we don't incorrectly 
% draw conclusions from the test, which may not actually be valid.
% One way of determining whether there is drift w.r.t. time in a particular 
% dataset is to fit a line (linear function) through the data, then run a
% Student's t-test on the slope parameter of the fit to determine if the
% variation of the slope parameter is greater than what we would expect to
% see if there were anything other than random variation in the dataset.

% Tests:
% 1. measures of location
% 2. confidence limits for the mean and one Sample t-Test
% 3. two sample t-Test for equal means
% 4. One factor analysis of variances
% 5. Multi-factor analysis of variance

% Step 1: Compute Line-of-Best-Fit using Least-Squares
k = 1:summary_stats.n; k = k'; % independent variable is time in discrete-time (DT)
Phi = [ones(summary_stats.n,1) k]; % assemble the "Phi" matrix for the dataset 
b = Phi \ data; % Perform least-squares minimization by taking "inverse" of "Phi" matrix w.r.t. the data
b0 = b(1); % the offset of the line of best fit (bias or mean)
b1 = b(2); % the slope of the line of best fit (ideally this is 0.0)

% Step 2: Compute the residual between the data and the line-of-best fit
residuals = data - (b1 * k + b0); % take the difference between the models expected output and the actual data
RSS = sum((residuals).^2); % compute the sum of the squares of the residuals to quantify the overall (unnormalized) error in the model
figure(1);histogram(residuals); title("Differences between Data and Line of Best Fit"); xlabel("Residual Magnitude"); ylabel("Frequency"); grid on; fontsize(gcf,scale=1.2);
figure(2); plot(k, data, 'k'); hold on; plot(k, k*b1 + b0, 'r--'); fontsize(gcf,scale=1.2); legend("Raw Data", "Line of Best-Fit"); title("Comparison of Raw Data and Line of Best Fit"); xlabel("Time [Samples]"); ylabel("Amplitude [V]"); grid on;
pause;

% Step 3: Compute the residuals degrees of freedom, std, and mean
res_dof = (summary_stats.n - size(Phi, 2));
res_std = sqrt(RSS / res_dof); % normalized quantification of how "well" the model fits the data (ideally 0)
mse_data = mean(residuals); % unnormalized, single value quantifying how "well" the model fit the data (ideally 0)

% Step 4: Compute the standard error of each of the line of best-fit parameters
se_data = sqrt(diag(res_std^2 * inv(Phi' * Phi))); % standard error of each coefficient

% Step 5: State a confidence level we want to use
alpha = 0.05; % 95% of the samples we take from the population, from which we derived this current sample (i.e. this current data set), will pass this test

% Step 6: Compute the critical value of the test statistic at this
% confidence level and degree of freedom
critical_value = icdf('t', (1-alpha/2), res_dof);

% Step 7: Compute the t-values for both parameters (we only care about the
% slope parameter, though)
tvalues_data = b ./ se_data;

% Step 8: Perform the hypothesis test
fprintf("\n2: Location:\n")
fprintf("H0: the slope is not different than 0.0\n")
fprintf("Ha: the slope is different than 0.0\n")

if abs(tvalues_data(2)) < critical_value
    fprintf("Do not reject H0 at %.2f significance level\n", alpha)
else
    fprintf("Reject H0 at %.2f significance level\n", alpha)
end

pause;

% Note:
% mdl = fitlm(k, data); % this line solves the problem in 1 command

%% Variation: 

% Tests (scale / variability / spread)
% 1. measures of scale
% 2. Bartlett's Test
% 3. Chi-Square Test
% 4. F-test
% 5. Levene Test

fprintf("\n3: Variation:\n")
fprintf("H0: sigma1 = sigma2 = sigma3 = sigma4\n");
fprintf("Ha: at least one sigmai is not equal to the others\n");
[T, crit_val] = bartlett_var_test(data, 4, alpha);

if T > crit_val
    fprintf("Reject H0 at %.2f significance level\n", alpha);
else
    fprintf("Do not reject H0 at %.2f significance level\n", alpha);
end

%% Distributional analysis
% is it normal?
% - probability plots (visual)
% - probability plot pearson coefficient (PPCC) (quantitative)
% - Chi-square and Kolmogorov-Smirnov goodness of fit tests
% - wilk-shapiro and anderson-darling test for normality (goodness of fit
% tests for normal distributions)
figure(3); normplot(data); fontsize(gcf,scale=1.2);
x = linspace(summary_stats.min, summary_stats.max);
figure(4); histogram(data, 'Normalization', 'pdf'); hold on; plot(x,pdf('normal', x, summary_stats.mean, summary_stats.std),'r-'); grid on; fontsize(gcf, scale=1.2); hold off;
figure(5); cdfplot(data); hold on; plot(x, cdf('Normal',x, summary_stats.mean, summary_stats.std), 'r-'); legend("Empirical CDF", "Hypothetical CDF", "Location", "Best"); grid on; fontsize(gcf, scale=1.2); hold off; 
pause;

% Anderson-Darling
Y = sort(data);
F = cdf('Normal', Y, summary_stats.mean, summary_stats.std); % change mu and sigma
S = 0;
for i = 1:summary_stats.n
    S = S + (2 * i - 1) / summary_stats.n * (log(F(i)) + log(1 - F(summary_stats.n+1-i)));
end

A_squared = -summary_stats.n - S;
crit_value = 0.787; %looked up table on Wikipedia

fprintf("\n\n4: Distributional Analysis:\n")
fprintf("H0: the data are normally distributed\n")
fprintf("Ha: the data are not normally distributed\n")

if A_squared > crit_value
    fprintf("Anderson-Darling: Reject H0\n")
else
    fprintf("Anderson-Darling: Do not reject H0\n")
end


% Kolmogorov-Smirnov Test
[h,p] = kstest((data-summary_stats.mean)/summary_stats.std);
if h == 0
    fprintf("Kolmogorov-Smirnov: Do not reject H0\n")
else
    fprintf("Kolmogorov-Smirnov: Reject H0\n")
end

%% Randomness
% - runs test
% - autocorrelation plot
% - lag plot
% - run sequence test
% - (the other matlab one) for independence?

%% Outlier analysis
% - grubbs test
% - Tietjen-Moore Test
% - Generalized Extreme deviate test
G = max(data - summary_stats.mean) / summary_stats.std;
dof = summary_stats.n - 2;
t_value = icdf('t', alpha / (2 * summary_stats.n), dof);
critical_value = (summary_stats.n-1) / sqrt(summary_stats.n) * sqrt(t_value^2 / (summary_stats.n - 2 + t_value^2));

fprintf("\n\n5: Outlier Analysis\n")
fprintf("H0: there are no outliers in the data\n")
fprintf("Ha: the maximum value is an outlier\n")

if G > critical_value
    fprintf("Grubbs' Test: Reject H0\n")
else
    fprintf("Grubbs' Test: Do not reject H0\n")
end

%% Model
% - interval the coefficients with 95% confidence intervals