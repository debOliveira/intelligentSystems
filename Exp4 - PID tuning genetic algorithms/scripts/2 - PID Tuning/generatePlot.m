clc
clear all

global xg yg v0 optionsODE len
len = 60;
xg=2; yg=2; v0=0.25; pd =50;
fg=atan2(yg,xg);
tfin=20;
xini=[eps;eps;-fg;eps;eps];
optionsODE=odeset('RelTol',1e-6);
%%
kp=0.1; ki=0.1; kd=0.01;
[t,x]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
load("data/IAEnewOrigin.mat");
[kp,ki,kd] = binArray2dec(eliteIndiv);
fprintf("IAE: kp=%.4f ki=%.4f kd=%.4f\n",kp,ki,kd);
[t0,x0]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
load("data/ISEnewOrigin.mat");
[kp,ki,kd] = binArray2dec(eliteIndiv);
fprintf("ISE: kp=%.4f ki=%.4f kd=%.4f\n",kp,ki,kd);
[t1,x1]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
load("data/ITAEnewOrigin.mat");
[kp,ki,kd] = binArray2dec(eliteIndiv);
fprintf("ITAE: kp=%.4f ki=%.4f kd=%.4f\n",kp,ki,kd);
[t2,x2]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
load("data/ITSEnewOrigin.mat");
[kp,ki,kd] = binArray2dec(eliteIndiv);
fprintf("ITSE: kp=%.4f ki=%.4f kd=%.4f\n",kp,ki,kd);
[t3,x3]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
%%
figure('Position', [0 0 600 400])
lineStyles = linspecer(5);
h(1)=plot(x(:,1),x(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(1,:));
hold on
for i=1:round(length(x(:,1))/30):length(x(:,1))
    desenherobo(x(i,1),x(i,2),x(i,3),0.03,lineStyles(1,:));
end
h(2)=plot(x0(:,1),x0(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(2,:));
for i=1:round(length(x0(:,1))/30):length(x0(:,1))
    desenherobo(x0(i,1),x0(i,2),x0(i,3),0.03,lineStyles(2,:));
end
h(3)=plot(x1(:,1),x1(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(3,:));
for i=1:round(length(x1(:,1))/30):length(x1(:,1))
    desenherobo(x1(i,1),x1(i,2),x1(i,3),0.03,lineStyles(3,:));
end
h(4)=plot(x2(:,1),x2(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(4,:));
for i=1:round(length(x2(:,1))/30):length(x2(:,1))
    desenherobo(x2(i,1),x2(i,2),x2(i,3),0.03,lineStyles(4,:));
end
h(5)=plot(x3(:,1),x3(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(5,:));
xlabel('x, (m)'), ylabel('y, (m)')
for i=1:round(length(x3(:,1))/30):length(x3(:,1))
    desenherobo(x3(i,1),x3(i,2),x3(i,3),0.03,lineStyles(5,:));
end
hold off
%%
axis equal
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR with PID control law}'},...
         'FontWeight','Normal');
legend(h([1 2 3 4 5]),"Original gains",...
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
function [kp,ki,kd]=binArray2dec(x)
    global len
    kp = hex2dec(binaryVectorToHex(x(:,1:len/3)))/(2^(len/3));
    ki = hex2dec(binaryVectorToHex(x(:,len/3+1:2*len/3)))/(2^(len/3));
    kd = hex2dec(binaryVectorToHex(x(:,2*len/3+1:len)))/(2^(len/3));
end

function xdot = smf(t,x,kp,ki,kd,pd)    
    global xg yg v0
    
    xp=x(1,1);
    yp=x(2,1);
    fp=x(3,1);
    z1=x(4,1);
    z2=x(5,1);

    fd=atan2(yg-yp,xg-xp);
    e=fd-fp;
    e=atan2(sin(e),cos(e));

    delta=norm([xg-xp yg-yp]);
    if delta < 0.01
        nu=0;
    else
        nu=v0;
    end
    xdot=[nu*cos(fp);nu*sin(fp);ki*pd*z1+(ki-kd*pd^2)*z2+(kp+kd*pd)*e;z2;-pd*z2+e];
end

function []=desenherobo(x,y,q,s,c)
    p=[ 1              1/7     
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