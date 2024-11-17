% Define the X-axis labels and data
X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});

% Data for time and path length (normalized)
Y_time_path = [time_avg(1,3), cost_avg(1,3);
    time_avg(2,3), cost_avg(2,3);
    time_avg(3,3), cost_avg(3,3);
    time_avg(4,3), cost_avg(4,3)];

figure;

% Left y-axis for Time
yyaxis left;
bar(X, [Y_time_path(:,1) NaN(size(Y_time_path,1), 1)]); % Left y-axis: Time
ylabel('Time (s)'); % Label Y-axis for Time

% Right y-axis for Path Length
yyaxis right;
bar(X, [NaN(size(Y_time_path,1), 1) Y_time_path(:,2)]); % Right y-axis: Path Length
ylabel('Path Length (m)'); % Label Y-axis for Path Length

% Set title and X-axis labels
title('Time and Path Length');
legend({'Time (s)', 'Path Length (m)'}, 'Location', 'northeast'); % Create legend
set(gca, 'FontSize', 15);
set(gca, 'LineWidth', 1);

% Customize X-axis labels
xticks(X);
xticklabels({'$\varepsilon = 1$', '$\varepsilon = 1.5$', '$\varepsilon = 2$', '$\varepsilon = 2.5$'});
set(gca, 'TickLabelInterpreter', 'latex');
