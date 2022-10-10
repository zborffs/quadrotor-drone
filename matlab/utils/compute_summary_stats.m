function out = compute_summary_stats(data)
%SUMMARY_STATS returns a struct of the summary statistics of input data
%   computes the number of elements, mean, median, max, min, range, and
%   sample standard deviation of the input data set... data set is assumed
%   to be single column of values

n_data = size(data, 1); % get all the rows of the column vector representing data set
mean_data = mean(data);
median_data = median(data);
max_data = max(data);
min_data = min(data);
range_data = max_data - min_data;
std_data = std(data);

out = struct( ...
    'n', n_data, ...
    'mean', mean_data, ...
    'median', median_data, ...
    'max', max_data, ...
    'min', min_data, ...
    'range', range_data, ...
    'std', std_data);

end

