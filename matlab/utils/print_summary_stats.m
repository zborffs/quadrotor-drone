function print_summary_stats(summary_stats)
%PRINT_SUMMARY_STATS prints to the summary stats object to the Command Window
%   usings fprintf(...) to prettily print summary stats to command window

format long; % anti-pattern (void functions shouldn't update global vars)

fprintf("Sample Size\t = %d\n", summary_stats.n)
fprintf("Mean\t\t = %d\n", summary_stats.mean)
fprintf("Median\t\t = %d\n", summary_stats.median)
fprintf("Min\t\t = %d\n", summary_stats.min)
fprintf("Max\t\t = %d\n", summary_stats.max)
fprintf("Range\t\t = %d\n", summary_stats.range)
fprintf("Stan. Dev\t = %d\n", summary_stats.std)

end

