%% ISD plot
Sec_ISD=ones(1+n,1);
figure()
hold on
pl4=plot([0;ISD_c_ATMD_DoS],[0:20],"Color",[127/255 128/255 255/255],'LineWidth',1.5);
pl1=plot([0;ISD_c_AMD],[0:20],'-.r','LineWidth',1.5);
pl2=plot([0;ISD_u_AMD],[0:20],'--k','LineWidth',1.5);
pl3=plot([0;ISD_c_ATMD],[0:20],'-x',"Color",[0/255 128/255 128/255],'LineWidth',1.5);
plot(Sec_ISD,[0:20],"--","Color",[245/255 222/255 179/255],'LineWidth',1.5);
hold off
legend([pl1 pl2 pl3 pl4],{'AMD controlled', 'AMD attacked','ATMD controlled', 'ATMD attacked'}) 
box on
xlabel("Maximum ISD ratio per story (\verb|%|)","Interpreter",'latex')
ylabel("Floor","Interpreter",'latex')
yticks([0:1:20]);
pbaspect([1.5 1 1])
set(gca,'FontSize',18)
yticks([0:1:20]);
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';

%% Time series
figure()
hold on
pl1=plot(t_AMD(1:2000),100*D_con_AMD(20,1:2000)/h(20),'-.r','LineWidth',1.5);
pl2=plot(t_AMD(1:2000),100*D_unc_AMD(20,1:2000)/h(20),'--k','LineWidth',1.5);
pl3=plot(t(1:2000),ones(length(t(1:2000)),1),"Color",[245/255 222/255 179/255],'LineWidth',1.5);
plot(t(1:2000),-ones(length(t(1:2000)),1),"Color",[245/255 222/255 179/255],'LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
ylim([-1.5 1.5])
xlabel("Time (s)","Interpreter",'latex')
ylabel("ISD (\verb|%|)","Interpreter",'latex')
legend([pl1 pl2],{'Controlled AMD system','Attacked AMD system'},"Interpreter",'latex')
set(gca,'FontSize',18)
%%
figure()
hold on
pl1=plot(t_con_ATMD(1:2000),100*D_con_ATMD(20,1:2000)/h(20),'-.',"Color",[0/255 128/255 128/255],'LineWidth',1.5);
pl2=plot(t_DoS_ATMD(1:2000),100*D_con_DoS_ATMD(20,1:2000)/h(20),"Color",[127/255 128/255 255/255],'LineWidth',1.5);
pl3=plot(t(1:2000),ones(length(t(1:2000)),1),"Color",[245/255 222/255 179/255],'LineWidth',1.5);
plot(t(1:2000),-ones(length(t(1:2000)),1),"Color",[245/255 222/255 179/255],'LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
ylim([-1.5 1.5])
xlabel("Time (s)","Interpreter",'latex')
ylabel("ISD (\verb|%|)","Interpreter",'latex')
legend([pl1 pl2],{'Controlled ATMD system','Attacked ATMD system'},"Interpreter",'latex')
set(gca,'FontSize',18)

%% SVD Hz
[s_unc_AMD,w_unc_AMD]=sigma(Sys_unc_AMD);
[s_con_AMD,w_con_AMD]=sigma(Sys_con_AMD);
[s_DoS_ATMD,w_DoS_ATMD]=sigma(Sys_DoS_ATMD);
[s_con_ATMD,w_con_ATMD]=sigma(Sys_con_ATMD);

figure()
hold on
% plot(w_con_AMD/(2*pi), 20*log10(s_con_AMD),'-.r','LineWidth',1.5)
% plot(w_unc_AMD/(2*pi), 20*log10(s_unc_AMD),'--k','LineWidth',1.5)
% plot(w_con_ATMD/(2*pi), 20*log10(s_con_ATMD),'-x',"Color",[0/255 128/255 128/255],'LineWidth',1.2)
% plot(w_DoS_ATMD/(2*pi), 20*log10(s_DoS_ATMD)+120,"Color",[127/255 128/255 255/255],'LineWidth',1.5)
% area(w_DoS_ATMD/(2*pi), 20*log10(s_DoS_ATMD)+120,'FaceColor',[139/255 0 0],'FaceAlpha',0.5,'LineWidth',2)
area(w_DoS_ATMD/(2*pi), 20*log10(s_DoS_ATMD)+120,'FaceColor',[150/256 18/256 36/256],'FaceAlpha',0.5,'LineWidth',2)
hold off
box on
grid on
pbaspect([1.5 1 1])
set(gca,'xscale','log')
set(gca,'FontSize',18)
set(gca,'xtick',[])
set(gca,'ytick',[])
xlabel('External disturbance frequency','Interpreter','latex');
ylabel('System ISD ratio response','Interpreter','latex');
% legend('AMD controlled', 'AMD attacked','ATMD controlled', 'ATMD attacked','Interpreter','latex') 
%% SVD rad/s
[s_unc_AMD,w_unc_AMD]=sigma(Sys_unc_AMD);
[s_con_AMD,w_con_AMD]=sigma(Sys_con_AMD);
[s_DoS_ATMD,w_DoS_ATMD]=sigma(Sys_DoS_ATMD);
[s_con_ATMD,w_con_ATMD]=sigma(Sys_con_ATMD);

figure()
hold on
semilogx(w_con_AMD, 20*log10(s_con_AMD),'-.r','LineWidth',1.5)
semilogx(w_unc_AMD, 20*log10(s_unc_AMD),'--k','LineWidth',1.5)
semilogx(w_con_ATMD, 20*log10(s_con_ATMD),'-x',"Color",[0/255 128/255 128/255],'LineWidth',1.2)
semilogx(w_DoS_ATMD, 20*log10(s_DoS_ATMD),"Color",[127/255 128/255 255/255],'LineWidth',1.5)
hold off
box on
grid on
pbaspect([1.5 1 1])
set(gca,'xscale','log')
set(gca,'FontSize',18)
xlabel('Frequency (rad/s)','Interpreter','latex');
ylabel('Singular Values (dB)','Interpreter','latex');
legend('AMD controlled', 'AMD attacked','ATMD controlled', 'ATMD attacked','Interpreter','latex') 