% cost_avg = cost_avg + costM;
% edge_avg = edge_avg + edgeM;
% time_avg = time_avg + timeM;

cost_avg = cost_avg/9;
edge_avg = edge_avg/9;
time_avg = time_avg/9;


% time_avg =
% 
%     2.3338    0.1464    0.1414    0.4640    0.8504    0.5125
%     0.7195    0.0476    0.0465    0.1087    0.1377    0.1201
%     0.4290    0.0292    0.0287    0.0512    0.0991    0.0707
%     0.3559    0.0253    0.0248    0.0426    0.1309    0.0744
% 
% cost_avg =
% 
%     9.0468    9.0468    9.0468    9.0468    9.0468    9.0468
%     9.0797    9.0797    9.0797    9.0797    9.0797    9.0797
%     9.2566    9.2566    9.2566    9.2564    9.2561    9.2561
%     9.4298    9.4298    9.4298    9.4296    9.4292    9.4292

% edge_avg =
% 
%    1.0e+03 *
% 
%     1.2651    0.1048    0.1048    0.0187    0.0541    0.0209
%     0.4284    0.0334    0.0334    0.0182    0.0273    0.0190
%     0.2678    0.0203    0.0203    0.0163    0.0211    0.0170
%     0.2292    0.0176    0.0176    0.0157    0.0206    0.0165


% 1265.09736669444	104.769619354706	104.769619354706	18.7111035248708	54.1089123213718	20.9436542578069
% 428.377273880204	33.4277861676493	33.4277861676493	18.2059079825388	27.3267227742030	19.0464662378038
% 267.830402078391	20.3130330945095	20.3130330945095	16.3136939187560	21.1028707444810	17.0061697057285
% 229.190909567509	17.6014089785500	17.6014089785500	15.6697639751056	20.6115084948898	16.5155767316042


X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [edge_avg(1,3), edge_avg(1,4)+0.5;
    edge_avg(2,3), edge_avg(2,4);
    edge_avg(3,3), edge_avg(3,4);
    edge_avg(4,3), edge_avg(4,4)]/edge_avg(1,4);
figure
b = bar(X,Y);
set(gca,'ytick',[])
title('Edge Evaluations')
legend('LEA*', 'LazySP')
set(gca,'FontSize',18);
set(gca,'LineWidth',1);
% ylim([0, 6])
s = {'$\varepsilon = 1$', '$\varepsilon = 1.5$', '$\varepsilon = 2$', '$\varepsilon =2.5$'};
set(gca,'xtick',X,'XTickLabel',s,'TickLabelInterpreter','latex');

xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(b(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

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
legend('Time', 'Path Length')
set(gca,'FontSize',18);
set(gca,'LineWidth',1);
ylim([0, 1.4])
s = {'$\varepsilon = 1$', '$\varepsilon = 1.5$', '$\varepsilon = 2$', '$\varepsilon =2.5$'};
set(gca,'xtick',X,'XTickLabel',s,'TickLabelInterpreter','latex');

xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(b(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

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