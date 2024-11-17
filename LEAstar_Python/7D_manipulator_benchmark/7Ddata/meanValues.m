data = all_data;
mean_1 = mean(data);
%% weight = 1
% time
% Astar LWAstar LEAstar LazySP_Astar LRAstar2 LRAstar4
time = [ mean_1(1) mean_1(5) mean_1(9) mean_1(13) mean_1(17) mean_1(21)];
% edge_count
edges = [ mean_1(2) mean_1(6) mean_1(10) mean_1(14) mean_1(18) mean_1(22)];
% vertex_count
vertices = [ mean_1(3) mean_1(7) mean_1(11) mean_1(15) mean_1(19) mean_1(23)];
% cost
cost = [ mean_1(4) mean_1(8) mean_1(12) mean_1(16) mean_1(20) mean_1(24)];

%% weight = 1.5
% time
time1 = [ mean_1(25) mean_1(29) mean_1(33) mean_1(37) mean_1(41) mean_1(45)];
% edge_count
edges1 =  [ mean_1(26) mean_1(30) mean_1(34) mean_1(38) mean_1(42) mean_1(46)];
% vertex_count
vertices1 =  [ mean_1(27) mean_1(31) mean_1(35) mean_1(39) mean_1(43) mean_1(47)];
% cost
cost1 =  [ mean_1(28) mean_1(32) mean_1(36) mean_1(40) mean_1(44) mean_1(48)];

%% weight = 2
% time
time2 = [ mean_1(49) mean_1(53) mean_1(57) mean_1(61) mean_1(65) mean_1(69)];
% edge_count
edges2 =  [ mean_1(50) mean_1(54) mean_1(58) mean_1(62) mean_1(66) mean_1(70)];
% vertex_count
vertices2 =  [ mean_1(51) mean_1(55) mean_1(59) mean_1(63) mean_1(67) mean_1(71)];
% cost
cost2 =  [ mean_1(52) mean_1(56) mean_1(60) mean_1(64) mean_1(68) mean_1(72)];

%% weight = 2.5
% time
time3 = [ mean_1(73) mean_1(77) mean_1(81) mean_1(85) mean_1(89) mean_1(93)];
% edge_count
edges3 = [ mean_1(74) mean_1(78) mean_1(82) mean_1(86) mean_1(90) mean_1(94)];
% vertex_count
vertices3 =  [ mean_1(75) mean_1(79) mean_1(83) mean_1(87) mean_1(91) mean_1(95)];
% cost
cost3 = [ mean_1(76) mean_1(80) mean_1(84) mean_1(88) mean_1(92) mean_1(96)];

for i = 1:10
[~,idx] = max(data(:,13));
data(idx, :) = [];
end
mean_2 = mean(data);
%% weight = 1
% time
% Astar LWAstar LEAstar LazySP_Astar LRAstar2 LRAstar4
time4 = [ mean_2(1) mean_2(5) mean_2(9) mean_2(13) mean_2(17) mean_2(21)];
% edge_count
edges4 = [ mean_2(2) mean_2(6) mean_2(10) mean_2(14) mean_2(18) mean_2(22)];
% vertex_count
vertices4 = [ mean_2(3) mean_2(7) mean_2(11) mean_2(15) mean_2(19) mean_2(23)];
% cost
cost4 = [ mean_2(4) mean_2(8) mean_2(12) mean_2(16) mean_2(20) mean_2(24)];

%% weight = 1.5
% time
time5 = [ mean_2(25) mean_2(29) mean_2(33) mean_2(37) mean_2(41) mean_2(45)];
% edge_count
edges5 =  [ mean_2(26) mean_2(30) mean_2(34) mean_2(38) mean_2(42) mean_2(46)];
% vertex_count
vertices5 =  [ mean_2(27) mean_2(31) mean_2(35) mean_2(39) mean_2(43) mean_2(47)];
% cost
cost5 =  [ mean_2(28) mean_2(32) mean_2(36) mean_2(40) mean_2(44) mean_2(48)];

%% weight = 2
% time
time6 = [ mean_2(49) mean_2(53) mean_2(57) mean_2(61) mean_2(65) mean_2(69)];
% edge_count
edges6 =  [ mean_2(50) mean_2(54) mean_2(58) mean_2(62) mean_2(66) mean_2(70)];
% vertex_count
vertices6 =  [ mean_2(51) mean_2(55) mean_2(59) mean_2(63) mean_2(67) mean_2(71)];
% cost
cost6 =  [ mean_2(52) mean_2(56) mean_2(60) mean_2(64) mean_2(68) mean_2(72)];

%% weight = 2.5
% time
time7 = [ mean_2(73) mean_2(77) mean_2(81) mean_2(85) mean_2(89) mean_2(93)];
% edge_count
edges7 = [ mean_2(74) mean_2(78) mean_2(82) mean_2(86) mean_2(90) mean_2(94)];
% vertex_count
vertices7 =  [ mean_2(75) mean_2(79) mean_2(83) mean_2(87) mean_2(91) mean_2(95)];
% cost
cost7 = [ mean_2(76) mean_2(80) mean_2(84) mean_2(88) mean_2(92) mean_2(96)];

timeM = [time4; time5; time6; time7];
edgeM = [edges4; edges5; edges6; edges7];
verticesM = [vertices4; vertices5; vertices6; vertices7];
costM = [cost4; cost5; cost6; cost7];


% X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
% X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
% Y = [time4; time5; time6; time7];
% figure
% b = bar(X,Y);
% set(gca,'ytick',[])
% ylabel('Time')
% legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=4')
% set(gca,'FontSize',15);
% set(gca,'LineWidth',1);
% 
% X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
% X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
% Y = [edges4; edges5; edges6; edges7];
% figure
% b = bar(X,Y);
% set(gca,'ytick',[])
% ylabel('Edges')
% legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=4')
% set(gca,'FontSize',15);
% set(gca,'LineWidth',1);
% 
% 
% X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
% X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
% Y = [vertices4; vertices5; vertices6; vertices7];
% figure
% b = bar(X,Y);
% set(gca,'ytick',[])
% ylabel('Vertices')
% legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=4')
% set(gca,'FontSize',15);
% set(gca,'LineWidth',1);
% 
% X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
% X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
% Y = [cost4; cost5; cost6; cost7];
% figure
% b = bar(X,Y);
% set(gca,'ytick',[])
% ylabel('Cost')
% legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=4')
% set(gca,'FontSize',15);
% set(gca,'LineWidth',1);

