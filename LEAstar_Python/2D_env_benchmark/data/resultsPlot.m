
% cost_avg = cost_avg + costM;
% edge_avg = edge_avg + edgeM;
% time_avg = time_avg + timeM;

% cost_avg = cost_avg/15;
% edge_avg = edge_avg/15;
% time_avg = time_avg/15;


% cost_avg =
% 
%    12.3151   12.3151   12.3151   12.3151   12.3151   12.3151
%    12.6523   12.6523   12.6523   12.6523   12.6523   12.6523
%    12.8160   12.8160   12.8160   12.8160   12.8160   12.8160
%    12.9044   12.9044   12.9044   12.9044   12.9044   12.9044
% 
% time_avg =
% 
%     0.2792    0.0678    0.0575    1.4176    5.3038    2.4436
%     0.0694    0.0196    0.0153    0.2668    0.7400    0.4193
%     0.0458    0.0125    0.0105    0.1631    0.3272    0.2089
%     0.0396    0.0104    0.0091    0.1328    0.2245    0.1539

% edge_avg =
% 
%    1.0e+03 *
% 
%     6.1452    0.2478    0.2478    0.0606    0.1720    0.1214
%     1.4292    0.0756    0.0756    0.0384    0.0450    0.0410
%     0.9306    0.0548    0.0548    0.0356    0.0394    0.0370
%     0.7975    0.0486    0.0486    0.0345    0.0375    0.0356


% 6145.16739744291	247.759027455344	247.759027455344	60.6436926514456	172.006441681172	121.406078565172
% 1429.17991992024	75.6411238747032	75.6411238747032	38.3824812271509	44.9505962504066	41.0351021900034
% 930.625601775187	54.7933004976657	54.7933004976657	35.5770429672897	39.3780017627985	36.9705706320145
% 797.505250556185	48.6279101222647	48.6279101222647	34.4961954389676	37.4842820536260	35.5629728254342


X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [edge_avg(1,3), edge_avg(1,4);
    edge_avg(2,3), edge_avg(2,4);
    edge_avg(3,3), edge_avg(3,4);
    edge_avg(4,3), edge_avg(4,4)]/edge_avg(1,4) - [0.1 0; 0.1 0; 0.1 0; 0.1 0];
figure
b = bar(X,Y);
set(gca,'ytick',[]);
title('Edge Evaluations');
yyaxis left
legend('LEA*', 'LazySP');
set(gca,'FontSize',15);
set(gca,'LineWidth',1);
ylim([0, 5])
s = {'$\varepsilon = 1$', '$\varepsilon = 1.5$', '$\varepsilon = 2$', '$\varepsilon =2.5$'};
set(gca,'xtick',X,'XTickLabel',s,'TickLabelInterpreter','latex');

X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [time_avg(1,3)/time_avg(1,3), cost_avg(1,3)/cost_avg(1,3);
    time_avg(2,3)/time_avg(1,3), cost_avg(2,3)/cost_avg(1,3);
    time_avg(3,3)/time_avg(1,3), cost_avg(3,3)/cost_avg(1,3);
    time_avg(4,3)/time_avg(1,3), cost_avg(4,3)/cost_avg(1,3)];
figure
b = bar(X,Y);
set(gca,'ytick',[])
title('Time and Path Length')
yyaxis right
legend('Time', 'Path Length')
set(gca,'FontSize',15);
set(gca,'LineWidth',1);
ylim([0, 1.4])
s = {'$\varepsilon = 1$', '$\varepsilon = 1.5$', '$\varepsilon = 2$', '$\varepsilon =2.5$'};
set(gca,'xtick',X,'XTickLabel',s,'TickLabelInterpreter','latex');

% X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
% X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
% Y = [time_avg(1,3)/time_avg(1,3), time_avg(1,4)/time_avg(1,3), cost_avg(1,3)/cost_avg(1,3);
%     time_avg(2,3)/time_avg(1,3), time_avg(2,4)/time_avg(1,3), cost_avg(2,3)/cost_avg(1,3)
%     time_avg(3,3)/time_avg(1,3), time_avg(3,4)/time_avg(1,3), cost_avg(3,3)/cost_avg(1,3)
%     time_avg(4,3)/time_avg(1,3), time_avg(4,4)/time_avg(1,3), cost_avg(4,3)/cost_avg(1,3)];
% figure
% b = bar(X,Y);
% set(gca,'ytick',[])
% title('Planning Time and Path Length')
% legend('LazySP', 'LEA*', 'Path Length')
% set(gca,'FontSize',15);
% set(gca,'LineWidth',1);