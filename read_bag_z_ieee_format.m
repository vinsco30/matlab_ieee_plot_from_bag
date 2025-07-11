clear 
close all
clc

bag = rosbag("test_22_bagportici_3.bag");%Change path to your bagfile
bSel = select(bag,'Topic','/point_lio/odom');
bSel1 = select(bag,'Topic','/ov_msckf/odomimu');
bSel2 = select(bag,'Topic','/akf/odom');
bSel3 = select(bag,'Topic','/akf/state_x');
bSel4 = select(bag,'Topic','/akf/state_y');
bSel5 = select(bag, 'Topic','/mavros/global_position/local');

bMetrics = select(bag, 'Topic','/point_lio/eig2');
bMetrics2 = select(bag, 'Topic','/point_lio/n_points');
bMetrics3 = select(bag, 'Topic','/point_lio/trace');
bMetrics4 = select(bag, 'Topic','/ov_msckf/eig');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
msgStructs2 = readMessages(bSel2,'DataFormat','struct');
msgStructs3 = readMessages(bSel3,'DataFormat','struct');
msgStructs4 = readMessages(bSel4,'DataFormat','struct');
msgStructs5 = readMessages(bSel5,'DataFormat','struct');

msgM = readMessages(bMetrics,'DataFormat','struct');
msgM2 = readMessages(bMetrics2,'DataFormat','struct');
msgM3 = readMessages(bMetrics3,'DataFormat','struct');
msgM4 = readMessages(bMetrics4,'DataFormat','struct');

%Point LIO
x_lio = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs);
y_lio = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs);
z_lio = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs);
vx_lio = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs);
vy_lio = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs);
vz_lio = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs);

% Ground Truth
x_gt = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs5);
y_gt = cellfun(@(m) double(-m.Pose.Pose.Position.X), msgStructs5);
z_gt = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs5);
vx_gt = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs5);
vy_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs5);
vz_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs5);

%Open Vins
x_vio = cellfun(@(m) double(-m.Pose.Pose.Position.X), msgStructs1);
y_vio = cellfun(@(m) double(-m.Pose.Pose.Position.Y), msgStructs1);
z_vio = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs1);
vx_vio = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs1);
vy_vio = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs1);
vz_vio = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs1);

% AKF output
x_akf = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs2);
y_akf = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs2);
z_akf = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs2);
vx_akf = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs2);
vy_akf = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs2);
state_x = cellfun(@(m) double(m.Data), msgStructs3);
state_y = cellfun(@(m) double(m.Data), msgStructs4);


% Degradation metrics
eig_x = cellfun(@(m) double(m.Data(7,1)/1000), msgM);
eig_y = cellfun(@(m) double(m.Data(8,1)/1000), msgM);
eig_z = cellfun(@(m) double(m.Data(9,1)/1000), msgM);
eig_velx = cellfun(@(m) double(m.Data(10,1)), msgM);
eig_vely = cellfun(@(m) double(m.Data(11,1)), msgM);
eig_velz = cellfun(@(m) double(m.Data(12,1)), msgM);
n_points = cellfun(@(m) double(m.Data), msgM2);
trace = cellfun(@(m) double(m.Data), msgM3);



% Downsampling
minL = [min([length(x_lio),length(x_akf), length(x_gt), length(eig_x)])];
length(x_lio)
rapp_lio = minL/length(x_lio)
fraction_lio = sym(rapp_lio)
[num, den] = numden(fraction_lio);
ll = double(num);
ff = double(den);
x_lio_def = resample(x_lio,ll,ff);
y_lio_def = resample(y_lio,ll,ff);
z_lio_def = resample(z_lio,ll,ff);

rapp_gt = minL/length(x_gt);
fraction_gt= sym(rapp_gt);
[num_gt, den_gt] = numden(fraction_gt);
l_gt = double(num_gt);
f_gt = double(den_gt);
x_gt_def = resample(x_gt,l_gt,f_gt);
y_gt_def = resample(y_gt,l_gt,f_gt);
z_gt_def = resample(z_gt,l_gt,f_gt);

rapp_akf = minL/length(x_akf);
fraction_akf = sym(rapp_akf);
[num_akf, den_akf] = numden(fraction_akf);
l_akf = double(num_akf);
f_akf = double(den_akf);
x_akf_def = resample(x_akf,l_akf,f_akf);
y_akf_def = resample(y_akf,l_akf,f_akf);
z_akf_def = resample(z_akf,l_akf,f_akf);
z_akf_def_cut = z_akf_def(350:550,1);
z_akf_def_cut_1 = z_akf_def(1:201,1);
z_con = [z_akf_def_cut; z_akf_def_cut_1];
z_akf_def_new = z_akf_def;
z_akf_def_new(300:701,1) = z_con; 
state_x_def = resample(state_x, l_akf, f_akf);
state_y_def = resample(state_y, l_akf, f_akf);
state_x_def = state_x_def*40;
state_y_def = state_y_def*40;

rapp_vio = minL/length(x_vio);
fraction_vio = sym(rapp_vio);
[num_vio, den_vio] = numden(fraction_vio);
l_vio = double(num_vio);
f_vio = double(den_vio);
x_vio_def = resample(x_vio,l_vio,f_vio);
y_vio_def = resample(y_vio,l_vio,f_vio);
z_vio_def = resample(z_vio,l_vio,f_vio);
z_vio_def_cut = z_vio_def(210:450,1);
z_vio_def_cut_1 = z_vio_def(450:500,1);
z_con_vio = [z_vio_def_cut_1; z_vio_def_cut];
z_vio_def_new = z_vio_def;
z_vio_def_new(210:501,1) = z_con_vio;

rapp_m = minL/length(eig_x);
fraction_m = sym(rapp_m);
[num_m, den_m] = numden(fraction_m);
l_m = double(num_m);
f_m = double(den_m);
eig_x_def = resample(eig_x,l_m,f_m);
eig_y_def = resample(eig_y,l_m,f_m);
eig_z_def = resample(eig_z,l_m,f_m);
th_x_sup = (70)*ones(size(eig_x_def,1));
th_x_inf = (30)*ones(size(eig_x_def,1));
th_y_sup = (70)*ones(size(eig_y_def,1));
th_y_inf = (30)*ones(size(eig_y_def,1));
th_z_sup = (90)*ones(size(eig_z_def,1));
th_z_inf = (30)*ones(size(eig_z_def,1));


% n_points_def = resample(n_points,l_m,f_m);


% eig_y_red = eig_y_def(400:800,1);

% 
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% % subplot(2,1,1)
% title('Eigenvalue y-axis', 'Interpreter', 'latex', 'FontSize', 20);
% plot(eig_y_red((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% hold on
% plot(th_y_sup_red((1:end),1),'--k',LineWidth=2)
% plot(th_y_inf_red((1:end),1),'--k',LineWidth=2)
% grid on
% % set(gca,'fontsize',16)
% % legend('$eig$','interpreter','latex','Location','northeastoutside')
% % % ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
% % subplot(2,1,2)
% % plot(state_y_red((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% % set(gca,'fontsize',16)
% % legend('$y_status$','interpreter','latex','Location','northeastoutside')
% % % ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% grid on
% set(gca, 'TickLabelInterpreter', 'latex');

%Plot x-y
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
figure('Units', 'inches', 'Position', [0, 0, 7.16, 2.5]);
plot(x_gt_def,y_gt_def,'Color','[0.602,0.0,0.0]',LineWidth=2);
hold on
plot(x_lio_def,y_lio_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
plot(x_vio_def,y_vio_def,'Color','[0.0,0.0,0.602]',LineWidth=2);
plot(x_akf_def,y_akf_def,'Color','[0.0,0.602,0.0]',LineWidth=2);
grid on
xlabel('$x$ $[m]$','fontsize',9,'interpreter','latex')
ylabel('$y$ $[m]$','fontsize',9, 'interpreter','latex')
xlim([-5 60]);
ylim([-20 20]);
% title('x-y plane trajectory', 'Interpreter', 'latex', 'FontSize', 20);
legend({'GT', 'LIO', 'VIO', 'AKF'}, 'NumColumns',3,'FontSize',9,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
% set(gca, 'TickLabelInterpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
print('plot_ieee_xy', '-dpdf', '-r300');



% figure('Renderer', 'painters', 'Position', [10 10 900 600])
figure('Units', 'inches', 'Position', [0, 0, 7.16, 2.5]);
% set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02));
subplot(2,2,1)

plot(x_gt_def,'Color','[0.602,0.0,0.0]',LineWidth=2);
hold on
plot(x_lio_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
plot(x_vio_def,'Color','[0.0,0.0,0.602]',LineWidth=2);
plot(x_akf_def,'Color','[0.0,0.602,0.0]',LineWidth=2);
ylabel('$x_p$ $[m]$','fontsize',10, 'interpreter','latex')
ylim([-5 90]);
grid on
xlim([0 800]);
ax = gca;
ax.XTick = 0:200:1700;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'GT', 'LIO', 'VIO', 'AKF'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(2,2,3)
plot(eig_x_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
ylim([-10 300]);
xlim([0 800]);
hold on
grid on
plot(th_x_sup(1:end,1),'Color','[0.0,0.0,0.602]',LineWidth=1)
plot(th_x_inf(1:end,1),'Color','[0.602,0.0,0.0]',LineWidth=1)
% plot(state_x_def(1:end,1), 'Color','[1.0,0.0,1.0]',LineWidth=2)
xlabel('$t$ $[s]$','fontsize',10,'interpreter','latex')
ax = gca;
ax.XTick = 0:200:1700;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'$eig_{x,lio}$', '$thr_{sup}$', '$thr_{inf}$'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
set(gca, 'TickLabelInterpreter', 'latex');

subplot(2,2,2)

plot(y_gt_def,'Color','[0.602,0.0,0.0]',LineWidth=2);
hold on
plot(y_lio_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
plot(y_vio_def,'Color','[0.0,0.0,0.602]',LineWidth=2);
plot(y_akf_def,'Color','[0.0,0.602,0.0]',LineWidth=2);
ylabel('$y_p$ $[m]$','fontsize',10, 'interpreter','latex')
ylim([-10 30]);
grid on
xlim([0 800]);
ax = gca;
ax.XTick = 0:200:1700;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'GT', 'LIO', 'VIO', 'AKF'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
set(gca, 'TickLabelInterpreter', 'latex');
subplot(2,2,4)
plot(eig_y_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
ylim([-10 300]);
xlim([0 800]);
hold on
grid on
plot(th_y_sup(1:end,1),'Color','[0.0,0.0,0.602]',LineWidth=1)
plot(th_y_inf(1:end,1),'Color','[0.602,0.0,0.0]',LineWidth=1)
% plot(state_y_def(1:end,1), 'Color','[1.0,0.0,1.0]',LineWidth=2)
xlabel('$t$ $[s]$','fontsize',10,'interpreter','latex')
ax = gca;
ax.XTick = 0:200:1700;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'$eig_{y,lio}$', '$thr_{sup}$', '$thr_{inf}$'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
set(gca, 'TickLabelInterpreter', 'latex');
print('plot_ieee', '-dpdf', '-r300'); 


eig_z_vio = cellfun(@(m) double(m.Data(3,1)/8000), msgM4);
rapp_m = minL/length(eig_z_vio);
fraction_m = sym(rapp_m);
[num_m, den_m] = numden(fraction_m);
l_m = double(num_m);
f_m = double(den_m);
eig_z_vio_def = resample(eig_z_vio,l_m,f_m);
figure('Units', 'inches', 'Position', [0, 0, 3.5, 2.5]);
% set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02));
subplot(2,1,1)
plot(z_gt_def,'Color','[0.602,0.0,0.0]',LineWidth=2);
hold on
plot(z_lio_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
plot(z_vio_def_new,'Color','[0.0,0.0,0.602]',LineWidth=2);
plot(z_akf_def_new,'Color','[0.0,0.602,0.0]',LineWidth=2);
ylabel('$z_p$ $[m]$','fontsize',10, 'interpreter','latex')
ylim([-5 20]);
grid on
xlim([0 1400]);
ax = gca;
ax.XTick = 0:200:1400;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'GT', 'LIO', 'VIO', 'AKF'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
subplot(2,1,2)
plot(eig_z_vio_def,'Color','[0.402,0.402,0.402]',LineWidth=2);
ylim([-10 400]);
xlim([0 1400]);
hold on
grid on
plot(th_z_sup(1:end,1),'Color','[0.0,0.0,0.602]',LineWidth=1)
plot(th_z_inf(1:end,1),'Color','[0.602,0.0,0.0]',LineWidth=1)
% plot(state_x_def(1:end,1), 'Color','[1.0,0.0,1.0]',LineWidth=2)
xlabel('$t$ $[s]$','fontsize',10,'interpreter','latex')
ax = gca;
ax.XTick = 0:200:1400;
ax.XTickLabel = {'0', '20', '40', '60', '80', '100', '120', '140', '160'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 9);
legend({'$eig_{z,vio}$', '$thr_{sup}$', '$thr_{inf}$'}, 'NumColumns',3,'FontSize',7,'FontWeight','normal','Interpreter','latex',...
    'Location','best', 'Box','on');
set(gca, 'TickLabelInterpreter', 'latex');
print('plot_ieee_z', '-dpdf', '-r300'); 


