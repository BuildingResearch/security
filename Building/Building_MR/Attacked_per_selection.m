%% Attacked floors per designed attack
figure()
hold on
for i=1:n
   for j=1:n
       if selecciones(i,j)==1
          scatter(i,j,"o","r",'filled'); 
       end
   end
   plot(i*ones(21,1),[0:20],"k",'LineWidth',1.5)
end

xlabel("Number of actuators disabled","Interpreter",'latex')
ylabel("Floor","Interpreter",'latex')
box on
pbaspect([1.5 1 1])
yticks([0:1:20]);
set(gca,'FontSize',15)
ax = gca;
ax.YGrid = 'on';
ax.GridLineStyle = '-';
%% Time response for ISD in the top floor
figure()
hold on
pl1=plot(t,100*D_con(20,:)/h(20),'--k','LineWidth',1.5);
pl2=plot(t_5,100*D_con_5(20,:)/h(20),'Color',[0/255 150/255 128/255],'LineWidth',1.5);
pl3=plot(t,ones(length(t),1),'-.r','LineWidth',1.5);
plot(t,-ones(length(t),1),'-.r','LineWidth',1.5)
hold off
box on
pbaspect([2 1 1])
ylim([-1.1 1.1])
xlabel("Time (s)","Interpreter",'latex')
ylabel("ISD (\verb|%|)","Interpreter",'latex')
legend([pl1 pl2],{'No actuators disabled','5 actuators disabled'},"Interpreter",'latex')
set(gca,'FontSize',18)
