%% SVD Hz
[s_FDI_AMD,w_FDI_AMD]=sigma(Sys_FDI_AMD);
[s_FDI_ATMD,w_FDI_ATMD]=sigma(Sys_FDI_ATMD);

figure()
hold on
semilogx(w_FDI_AMD/(2*pi), 20*log10(s_FDI_AMD),'--k','LineWidth',1.5)
semilogx(w_FDI_ATMD/(2*pi), 20*log10(s_FDI_ATMD),"Color",[0/255 128/255 128/255],'LineWidth',1.5)
hold off
box on
grid on
pbaspect([1.5 1 1])
set(gca,'xscale','log')
set(gca,'FontSize',18)
xlabel('Frequency (Hz)',"Interpreter",'latex');
ylabel('Singular Values (dB)',"Interpreter",'latex');
legend('AMD', 'ATMD',"Interpreter",'latex') 
%% SVD rad/s
[s_FDI_AMD,w_FDI_AMD]=sigma(Sys_FDI_AMD);
[s_FDI_ATMD,w_FDI_ATMD]=sigma(Sys_FDI_ATMD);

figure()
hold on
semilogx(w_FDI_AMD, 20*log10(s_FDI_AMD),'--k','LineWidth',1.5)
semilogx(w_FDI_ATMD, 20*log10(s_FDI_ATMD),"Color",[0/255 128/255 128/255],'LineWidth',1.5)
hold off
box on
grid on
pbaspect([1.5 1 1])
set(gca,'xscale','log')
set(gca,'FontSize',18)
xlabel('Frequency (rad/s)',"Interpreter",'latex');
ylabel('Singular Values (dB)',"Interpreter",'latex');
legend('AMD', 'ATMD',"Interpreter",'latex') 

%% Max ISD plot

Sec_ISD=ones(1+n,1);
figure()
hold on
plot([0;ISD_c_AMD],[0:20],'--k','LineWidth',1.5)
plot([0;ISD_c_ATMD],[0:20],"Color",[0/255 128/255 128/255],'LineWidth',1.5)
plot(Sec_ISD,[0:20],"-.","Color","r",'LineWidth',1.5);
hold off
legend('AMD','ATMD',"Interpreter",'latex') 
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
pl1=plot(t_AMD,100*D_con_AMD(20,:)/h(20),'--k','LineWidth',1.5);
pl2=plot(t_con_ATMD,100*D_con_ATMD(20,:)/h(20),'Color',[0/255 150/255 128/255],'LineWidth',1.5);
pl3=plot(t,ones(length(t),1),'-.r','LineWidth',1.5);
plot(t,-ones(length(t),1),'-.r','LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
xlabel("Time (s)","Interpreter",'latex')
xlim([0 t(end)])
ylabel("ISD (\verb|%|)","Interpreter",'latex')
legend([pl1 pl2],{'AMD','ATMD'},"Interpreter",'latex')
set(gca,'FontSize',18)
%% Actuator force
figure()
hold on
pl1=plot(t_con_ATMD(1:2100),1000e5*sin(0.284687*2*pi*t_con_ATMD(1:2100))/(10^6),'--k','LineWidth',1.5);
hold off
box on
pbaspect([2 1 1])
xlabel("Time (s)","Interpreter",'latex')
xlim([0 t(2000)])
ylabel("Force (MN)","Interpreter",'latex')
legend([pl1],{'AMD'},"Interpreter",'latex')
set(gca,'FontSize',18)

figure()
hold on
pl2=plot(t_con_ATMD(1:2100),F_act_ATMD(1:2100)/(10^6),'Color',[0/255 150/255 128/255],'LineWidth',1.5);
hold off
box on
pbaspect([2 1 1])
xlabel("Time (s)","Interpreter",'latex')
xlim([0 t(2000)])
ylabel("Force (MN)","Interpreter",'latex')
legend([pl2],{'ATMD'},"Interpreter",'latex')
set(gca,'FontSize',18)

