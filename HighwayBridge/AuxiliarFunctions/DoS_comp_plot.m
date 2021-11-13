function [] = DoS_comp_plot(At_test,ISD_test,t,n_act,n)

% Function to compare optimal attack with k actuators disabled
% with other possible combinations
t_out=t;

ISD_test=ISD_test*100;
[nt,n]=size(At_test);
figure('Position',[100,100,1000,500])

safety=13.8;
subplot(4,4,[1 2 3 5 6 7 9 10 11 13 14 15])
hold on
pl1=plot(t_out,ISD_test(1,:),'k','LineWidth',1.5);
pl2=plot(t_out,ISD_test(2,:),'--',"Color",[0/255 128/255 128/255],'LineWidth',1.5);
pl3=plot(t_out,ISD_test(3,:),'-.',"Color",[127/255 128/255 255/255],'LineWidth',1.2);
pl4=plot(t_out,ones(length(t_out),1)*safety,'-.r','LineWidth',1.5);
plot(t_out,-ones(length(t_out),1)*safety,'-.r','LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
ylim([-1.2 1.2]*safety)
xlim([0 t_out(end)])
xlabel("Time (s)","Interpreter",'latex')
ylabel("Displacement(cm)","Interpreter",'latex')
legend([pl1 pl2 pl3],{'No actuators disabled','Optimal attack to 6 actuators','Random attack to 6 actuators'},"Interpreter",'latex')
set(gca,'FontSize',18)
fl_sc=[];
ISD_sc=[];

for i=1:nt
    for j=1:n
        if At_test(i,j)==1
            fl_sc=[fl_sc j];
            ISD_sc=[ISD_sc i]; 
        end
    end
end

subplot(4,4,[4 8 12 16])
hold on
pl3=scatter(ISD_sc,fl_sc,50,"o","r",'filled','DisplayName','Attacked floors');
plot(ones(21,1),[0:20],'k','LineWidth',1.5)
plot(2*ones(21,1),[0:20],'--',"Color",[0/255 128/255 128/255],'LineWidth',1.5)
plot(3*ones(21,1),[0:20],'-.',"Color",[127/255 128/255 255/255],'LineWidth',1.5)
hold off
xlim([0.5 3.5])
yticks([0:1:20]);
pbaspect([1 2.4 1])
set(gca,'xtick',[])
set(gca,'FontSize',18)
set(gca,'xticklabel',[])
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';
end
