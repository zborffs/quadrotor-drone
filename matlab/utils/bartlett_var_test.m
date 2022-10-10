function [T, critical_value] = bartlett_var_test(data, num_groups, alpha)
%BARTLETT_VAR_TEST compute Bartlett's of equal variance on a single dataset
%to be partitioned into 'num_groups' groups at the signficance level of
%'alpha'
%   compute the T statistic and the critical value of the test

assert(size(data, 2) == 1); % assume data is nx1 vector of numbers
assert(mod(size(data, 1), num_groups) == 0, 'Cannot partition data into \"num_groups\" groups of equal size');

N = size(data, 1);
k = num_groups;

% partition 'data' into equally-sized 'num_groups' groups, compute
% variances of each subgroup
var_groups = [];
group_size = N / k;
for i = 1:k
    starter_index = (i-1)*group_size + 1;
%     x = [x data(starter_index:i*group_size)]; % very computationally inefficient solution
    var_groups = [var_groups var(data(starter_index:i*group_size))];
end

% compute the variance of each column
var_pooled = (group_size-1) / (N-k) * sum(var_groups); % allowed b/c group_size is same for all groups

% compute statistic T
T = ( (N-k) * log(var_pooled) - (group_size - 1) * sum(log(var_groups)) ) / (1 + (1/(3*(k-1))) * (sum(1/(group_size - 1)) - 1 / (N - k)));

dof = k - 1;
critical_value = icdf('Chisquare', (1-alpha), dof);

end

