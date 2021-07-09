clc
clear all

%%
load("data/IAE.mat");
x=[xp,yp,fp];
load("data/ISE.mat");
x0=[xp,yp,fp];
load("data/ITAE.mat");
x1=[xp,yp,fp];
load("data/ITSE.mat");
x2=[xp,yp,fp];
%%
figure('Position', [0 0 600 400])
lineStyles = linspecer(5);
h(1)=plot(x(:,1),x(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(2,:));
hold on
for i=1:round(length(x(:,1))/30):length(x(:,1))
    desenherobo(x(i,1),x(i,2),x(i,3),0.03,lineStyles(2,:));
end
h(2)=plot(x0(:,1),x0(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(3,:));
for i=1:round(length(x0(:,1))/30):length(x0(:,1))
    desenherobo(x0(i,1),x0(i,2),x0(i,3),0.03,lineStyles(3,:));
end
h(3)=plot(x1(:,1),x1(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(4,:));
for i=1:round(length(x1(:,1))/30):length(x1(:,1))
    desenherobo(x1(i,1),x1(i,2),x1(i,3),0.03,lineStyles(4,:));
end
h(4)=plot(x2(:,1),x2(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(5,:));
for i=1:round(length(x2(:,1))/30):length(x2(:,1))
    desenherobo(x2(i,1),x2(i,2),x2(i,3),0.03,lineStyles(5,:));
end
hold off
%%
axis equal
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR with PID control law}'},...
         'FontWeight','Normal');
legend(h([1 2 3 4]),...
    "IAE",'ISE','ITAE',"ITSE","Location","southeast")
grid
hold off

%%
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

[~,~,~] = mkdir('pics');
nameFile="all";
saveas(gca,"pics/"+nameFile+".png")
%%
function []=desenherobo(x,y,q,s,c)
    p=2.5*[ 1              1/7     
       -3/7            1       
       -5/7            6/7     
       -5/7            5/7     
       -3/7            2/7     
       -3/7            0       
       -3/7           -2/7     
       -5/7           -5/7     
       -5/7           -6/7     
       -3/7           -1       
        1             -1/7     
        1              1/7 ];
    p=s*p;
    p=[p,ones(length(p),1)];
    r=[cos(q),sin(q);-sin(q),cos(q);x,y];
    p=p*r;
    X=p(:,1); 
    Y=p(:,2); 
    plot(X,Y,'-','color',c,'LineWidth',1.0)
end