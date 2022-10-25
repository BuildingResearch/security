% This script generates a plot showing what actuators are attacked for
% k=1,...,20

selec=selecciones(2:21,:);
figure()
hold on
for i=1:n
   for j=1:n
       if selec(i,j)==1
          scatter(i,j,"o","r",'filled'); 
       end
   end
   plot(i*ones(21,1),[0:20],"k",'LineWidth',1.5)
end

xlabel("Number of actuators disabled","Interpreter",'latex')
ylabel("Actuator","Interpreter",'latex')
box on
pbaspect([1.5 1 1])
yticks([0:1:20]);
set(gca,'FontSize',15)
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';

safety=13.8; % cm
figure()
hold on
pl1=plot(t_out,100*D_con(:,2),'--k','LineWidth',1.5);
pl2=plot(t_out,100*D_con_6(:,2),'Color',[0/255 150/255 128/255],'LineWidth',1.5);
pl3=plot(t_out,ones(length(t_out),1)*safety,'-.r','LineWidth',1.5);
plot(t_out,-ones(length(t_out),1)*safety,'-.r','LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
ylim([-1.1 1.1]*safety)
xlabel("Time (s)","Interpreter",'latex')
ylabel("Displacement(cm)","Interpreter",'latex')
legend([pl1 pl2],{'No actuators disabled','6 actuators disabled'},"Interpreter",'latex')
set(gca,'FontSize',18)
