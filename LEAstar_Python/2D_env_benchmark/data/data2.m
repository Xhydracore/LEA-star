% [t_Astar, e_c1, v_c1, cost1, t_LWAstar, e_c2, v_c2, cost2, t_LazyEAstar, e_c3, v_c3, cost3, t_LazySP_Astar, e_c4, v_c4, cost4, t_LRAstar, e_c5, v_c5, cost5, t_LRAstar_1, e_c6, v_c6, cost6,
% t_Astar1, e_c11, v_c11, cost11, t_LWAstar1, e_c12, v_c12, cost12, t_LazyEAstar1, e_c13, v_c13, cost13, t_LazySP_Astar1, e_c14, v_c14, cost14, t_LRAstar1, e_c15, v_c15, cost15, t_LRAstar1_1, e_c16, v_c16, cost16,
% t_Astar2, e_c21, v_c21, cost21, t_LWAstar2, e_c22, v_c22, cost22, t_LazyEAstar2, e_c23, v_c23, cost23, t_LazySP_Astar2, e_c24, v_c24, cost24, t_LRAstar2, e_c25, v_c25, cost25, t_LRAstar2_1, e_c26, v_c26, cost26,
% t_Astar3, e_c31, v_c31, cost31, t_LWAstar3, e_c32, v_c32, cost32, t_LazyEAstar3, e_c33, v_c33, cost33, t_LazySP_Astar3, e_c34, v_c34, cost34, t_LRAstar3, e_c35, v_c35, cost35, t_LRAstar3_1, e_c36, v_c36, cost36]

%% weight = 1
% time
Astar_time=data(:, 1);
LWAstar_time=data(:, 5);
LEAstar_time=data(:, 9);
LazySP_Astar_time=data(:, 13);
LRAstar_time=data(:, 17);
LRAstar_time_1=data(:, 21);
% edge_count
Astar_e=data(:, 2);
LWAstar_e=data(:, 6);
LEAstar_e=data(:, 10);
LazySP_Astar_e=data(:, 14);
LRAstar_e=data(:, 18);
LRAstar_e_1=data(:, 22);
% vertex_count
Astar_v=data(:, 3);
LWAstar_v=data(:, 7);
LEAstar_v=data(:, 11);
LazySP_Astar_v=data(:, 15);
LRAstar_v=data(:, 19);
LRAstar_v_1=data(:, 23);
% cost
Astar_c=data(:, 4);
LWAstar_c=data(:, 8);
LEAstar_c=data(:, 12);
LazySP_Astar_c=data(:, 16);
LRAstar_c=data(:, 20);
LRAstar_c_1=data(:, 24);

%% weight = 1.5
% time
Astar_time1=data(:, 25);
LWAstar_time1=data(:, 29);
LEAstar_time1=data(:, 33);
LazySP_Astar_time1=data(:, 37);
LRAstar_time1=data(:, 41);
LRAstar_time1_1=data(:, 45);
% edge_count
Astar_e1=data(:, 26);
LWAstar_e1=data(:, 30);
LEAstar_e1=data(:, 34);
LazySP_Astar_e1=data(:, 38);
LRAstar_e1=data(:, 42);
LRAstar_e1_1=data(:, 46);
% vertex_count
Astar_v1=data(:, 27);
LWAstar_v1=data(:,31);
LEAstar_v1=data(:, 35);
LazySP_Astar_v1=data(:, 39);
LRAstar_v1=data(:, 43);
LRAstar_v1_1=data(:, 47);
% cost
Astar_c1=data(:, 28);
LWAstar_c1=data(:,32);
LEAstar_c1=data(:, 36);
LazySP_Astar_c1=data(:, 40);
LRAstar_c1=data(:, 44);
LRAstar_c1_1=data(:, 48);

%% weight = 2
% time
Astar_time2=data(:, 49);
LWAstar_time2=data(:, 53);
LEAstar_time2=data(:, 57);
LazySP_Astar_time2=data(:, 61);
LRAstar_time2=data(:, 65);
LRAstar_time2_1=data(:, 69);
% edge_count
Astar_e2=data(:, 50);
LWAstar_e2=data(:, 54);
LEAstar_e2=data(:, 58);
LazySP_Astar_e2=data(:, 62);
LRAstar_e2=data(:, 66);
LRAstar_e2_1=data(:, 70);
% vertex_count
Astar_v2=data(:, 51);
LWAstar_v2=data(:, 55);
LEAstar_v2=data(:, 59);
LazySP_Astar_v2=data(:, 63);
LRAstar_v2=data(:, 67);
LRAstar_v2_1=data(:, 71);
% cost
Astar_c2=data(:, 52);
LWAstar_c2=data(:, 56);
LEAstar_c2=data(:, 60);
LazySP_Astar_c2=data(:, 64);
LRAstar_c2=data(:, 68);
LRAstar_c2_1=data(:, 72);

%% weight = 2.5
% time
Astar_time3=data(:, 73);
LWAstar_time3=data(:, 77);
LEAstar_time3=data(:, 81);
LazySP_Astar_time3=data(:, 85);
LRAstar_time3=data(:, 89);
LRAstar_time3_1=data(:, 93);
% edge_count
Astar_e3=data(:, 74);
LWAstar_e3=data(:, 78);
LEAstar_e3=data(:, 82);
LazySP_Astar_e3=data(:, 86);
LRAstar_e3=data(:, 90);
LRAstar_e3_1=data(:, 94);
% vertex_count
Astar_v3=data(:, 75);
LWAstar_v3=data(:, 79);
LEAstar_v3=data(:, 83);
LazySP_Astar_v3=data(:, 87);
LRAstar_v3=data(:, 91);
LRAstar_v3_1=data(:, 95);
% cost
Astar_c3=data(:, 76);
LWAstar_c3=data(:, 80);
LEAstar_c3=data(:, 84);
LazySP_Astar_c3=data(:, 88);
LRAstar_c3=data(:, 92);
LRAstar_c3_1=data(:, 96);

for i = 1:10
    [~,idx] = max(Astar_time);
    Astar_time(idx) = [];
    [~,idx] = max(LWAstar_time);
    LWAstar_time(idx) = [];
    [~,idx] = max(LEAstar_time);
    LEAstar_time(idx) = [];
    [~,idx] = max(LazySP_Astar_time);
    LazySP_Astar_time(idx) = [];
    [~,idx] = max(LRAstar_time);
    LRAstar_time(idx) = [];
    [~,idx] = max(LRAstar_time_1);
    LRAstar_time_1(idx) = [];
    [~,idx] = max(Astar_time1);
    Astar_time1(idx) = [];
    [~,idx] = max(LWAstar_time1);
    LWAstar_time1(idx) = [];
    [~,idx] = max(LEAstar_time1);
    LEAstar_time1(idx) = [];
    [~,idx] = max(LazySP_Astar_time1);
    LazySP_Astar_time1(idx) = [];
    [~,idx] = max(LRAstar_time1);
    LRAstar_time1(idx) = [];
    [~,idx] = max(LRAstar_time1_1);
    LRAstar_time1_1(idx) = [];
end

figure
boxplot([Astar_time, LWAstar_time, LEAstar_time, LazySP_Astar_time, LRAstar_time, LRAstar_time_1],'Labels',{'A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=3'})
title('w = 1')
figure
boxplot([Astar_time1, LWAstar_time1, LEAstar_time1, LazySP_Astar_time1, LRAstar_time1, LRAstar_time1_1],'Labels',{'A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=3'})
title('w = 1.5')

X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [mean(Astar_time) mean(LWAstar_time) mean(LEAstar_time) mean(LazySP_Astar_time) mean(LRAstar_time) mean(LRAstar_time_1); 
    mean(Astar_time1) mean(LWAstar_time1) mean(LEAstar_time1) mean(LazySP_Astar_time1) mean(LRAstar_time1) mean(LRAstar_time1_1);
    mean(Astar_time2) mean(LWAstar_time2) mean(LEAstar_time2) mean(LazySP_Astar_time2) mean(LRAstar_time2) mean(LRAstar_time2_1)
    mean(Astar_time3) mean(LWAstar_time3) mean(LEAstar_time3) mean(LazySP_Astar_time3) mean(LRAstar_time3) mean(LRAstar_time3_1)];
figure
bar(X,Y);
ylabel('Time (s)')
legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=3')
set(gca,'FontSize',15);
set(gca,'LineWidth',1);

X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [mean(Astar_e) mean(LWAstar_e) mean(LEAstar_e) mean(LazySP_Astar_e) mean(LRAstar_e) mean(LRAstar_e_1); 
    mean(Astar_e1) mean(LWAstar_e1) mean(LEAstar_e1) mean(LazySP_Astar_e1) mean(LRAstar_e1) mean(LRAstar_e1_1);
    mean(Astar_e2) mean(LWAstar_e2) mean(LEAstar_e2) mean(LazySP_Astar_e2) mean(LRAstar_e2) mean(LRAstar_e2_1)
    mean(Astar_e3) mean(LWAstar_e3) mean(LEAstar_e3) mean(LazySP_Astar_e3) mean(LRAstar_e3) mean(LRAstar_e3_1)];
figure
b = bar(X,Y);
set(gca,'ytick',[])
ylabel('Edges')
legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=3')
set(gca,'FontSize',15);
set(gca,'LineWidth',1);

% xtips1 = b(1).XEndPoints;
% ytips1 = b(1).YEndPoints;
% labels1 = string(b(1).YData);
% text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips2 = b(2).XEndPoints;
% ytips2 = b(2).YEndPoints;
% labels2 = string(b(2).YData);
% text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips3 = b(3).XEndPoints;
% ytips3 = b(3).YEndPoints;
% labels3 = string(b(3).YData);
% text(xtips3,ytips3,labels3,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips4 = b(4).XEndPoints;
% ytips4 = b(4).YEndPoints;
% labels4 = string(b(4).YData);
% text(xtips4,ytips4,labels4,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')


X = categorical({'w=1','w=1.5', 'w=2','w=2.5'});
X = reordercats(X,{'w=1','w=1.5', 'w=2','w=2.5'});
Y = [mean(Astar_v) mean(LWAstar_v) mean(LEAstar_v) mean(LazySP_Astar_v) mean(LRAstar_v) mean(LRAstar_v_1); 
    mean(Astar_v1) mean(LWAstar_v1) mean(LEAstar_v1) mean(LazySP_Astar_v1) mean(LRAstar_v1) mean(LRAstar_v1_1);
    mean(Astar_v2) mean(LWAstar_v2) mean(LEAstar_v2) mean(LazySP_Astar_v2) mean(LRAstar_v2) mean(LRAstar_v2_1)
    mean(Astar_v3) mean(LWAstar_v3) mean(LEAstar_v3) mean(LazySP_Astar_v3) mean(LRAstar_v3) mean(LRAstar_v3_1)];
figure
b = bar(X,Y);
set(gca,'ytick',[])
ylabel('Vertices')
legend('A*', 'LWA*', 'LEA*', 'LazySP', 'LRA* d=2', 'LRA* d=3')
set(gca,'FontSize',15);
set(gca,'LineWidth',1);

% xtips1 = b(1).XEndPoints;
% ytips1 = b(1).YEndPoints;
% labels1 = string(b(1).YData);
% text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips2 = b(2).XEndPoints;
% ytips2 = b(2).YEndPoints;
% labels2 = string(b(2).YData);
% text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips3 = b(3).XEndPoints;
% ytips3 = b(3).YEndPoints;
% labels3 = string(b(3).YData);
% text(xtips3,ytips3,labels3,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
% xtips4 = b(4).XEndPoints;
% ytips4 = b(4).YEndPoints;
% labels4 = string(b(4).YData);
% text(xtips4,ytips4,labels4,'HorizontalAlignment','center',...
%     'VerticalAlignment','bottom')
