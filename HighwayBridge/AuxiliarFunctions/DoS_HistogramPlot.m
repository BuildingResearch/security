%% Attack selection histogram
% This script generates a histogram of the actuators that are disabled for
% k=1,...,20
load('optimalAttacks.mat')

S = Selection(2:21,:);
S=S.*[1:20];

figure('Position',[100 100 900 400])
box on
% pbaspect([1.5 1 1])

histogram(S(:),'LineWidth',1.5)
xlim([0.5 20.5])
ylim([0 20])
xlabel("Actuator","Interpreter",'latex')
ylabel(["Number of times", "being disabled"],"Interpreter",'latex')
xticks([1:1:20]);
set(gca,'FontSize',18)
set(gca,'linewidth',1.5)