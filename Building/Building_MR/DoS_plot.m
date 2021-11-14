function [] = DoS_plot(n,ISD_DoS,h2_norm)
% Plot for the MAX_ISD and h2 norm per DoS attack
% n=20;
% ISD_DoS = 5*rand(n,n);
% h2_norm=0.5*rand(n,1);
% 
% DoS_plot(n,ISD_DoS,h2_norm)

for i=1:20
   Max_ISD_DoS(i)=max(ISD_DoS(i,:)); 
end
Sec_ISD=ones(n+2,1);

figure()
hold on
bar([1:n],Max_ISD_DoS,"FaceColor",[0,0.7,0.9])
plot([0:n+1],Sec_ISD,"r--","LineWidth",2)
hold off
box on
pbaspect([2 1 1])
% legend("ISD ratios", "Max secure ISD ratio","Interpreter",'latex')
xlabel("Number of actuators disabled","Interpreter",'latex')
xlim([0 21])
ylabel("Overall Maximum ISD ratio (\verb|%|)","Interpreter",'latex')
set(gca,'FontSize',18)

figure()
hold on
plot([1:20],20*log10(h2_norm),"LineWidth",1.5)
scatter([1:20],20*log10(h2_norm),75,"filled","r")
hold off
box on
pbaspect([2 1 1])
xlabel("Number of actuators disabled","Interpreter",'latex')
ylabel("H2 norm (dB)","Interpreter",'latex')
set(gca,'FontSize',18)
end
