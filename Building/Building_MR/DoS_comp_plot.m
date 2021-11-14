function [] = DoS_comp_plot(At_test,ISD_test,n_act,n)

% Function to compare optimal attack with k actuators disabled
% with other possible combinations

% n_act=5;
% 
% FDI_comp_plot(At_test,ISD_test,n_act,n)

[nt,n]=size(At_test);
figure()
subplot(4,4,[1 2 3 5 6 7 9 10 11 13 14 15])
ISD_otp=ISD_test(1,:);
ISD_non_opt=ISD_test(2:end,:);
Sec_ISD=ones(1+n,1);
fl_sc=[];
ISD_sc=[];

for i=1:nt
    for j=1:n
        if At_test(i,j)==1
            fl_sc=[fl_sc j];
            ISD_sc=[ISD_sc i]; 
%             ISD_sc=[ISD_sc ISD_test(i,j)]; 
        end
    end
end
hold on
pl1=plot([0 ISD_otp],[0:20],"k",'LineWidth',1.5,'DisplayName','Optimal attack)');
for i = 1:nt-1
    if i==1
        pl2=plot([0 ISD_non_opt(i,:)],[0:20],'-.',"Color",[0/255 128/255 128/255],'LineWidth',1.5,'DisplayName','Other attacks');
    else 
        pl3=plot([0 ISD_non_opt(i,:)],[0:20],':',"Color",[127/255 128/255 255/255],'LineWidth',1.5);
    end
end
% pl3=scatter(ISD_sc,fl_sc,50,"o","r",'filled','DisplayName','Attacked floors');
pl4=plot(Sec_ISD,[0:20],"--","Color","r",'DisplayName','Maximum secure ISD','LineWidth',1.5);
hold off
xlabel("Maximum ISD ratio per story (\verb|%|)","Interpreter",'latex','FontSize',20)
ylabel("Floor","Interpreter",'latex','FontSize',20)
box on
pbaspect([1.5 1 1])
set(gca,'FontSize',18)
yticks([0:1:20]);
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';
% legend([pl1 pl2 pl3 pl4],{"Optimal attack", "Other attacks","Attacked floors","Maximum secure ISD"},"Interpreter",'latex')
legend([pl1 pl2 pl3],{"Optimal attack", "First five floors attacked","Random attack"},"Interpreter",'latex')

subplot(4,4,[4 8 12 16])
hold on
pl3=scatter(ISD_sc,fl_sc,50,"o","r",'filled','DisplayName','Attacked floors');
plot(ones(21,1),[0:20],'k','LineWidth',1.5)
plot(2*ones(21,1),[0:20],"Color",[0/255 128/255 128/255],'LineWidth',1.5)
plot(3*ones(21,1),[0:20],"Color",[127/255 128/255 255/255],'LineWidth',1.5)
hold off
xlim([0.5 3.5])
yticks([0:1:20]);
pbaspect([1 2.4 1])
% ylabel('Floors',"Interpreter",'latex')
set(gca,'xtick',[])
set(gca,'FontSize',18)
set(gca,'xticklabel',[])
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';
end
