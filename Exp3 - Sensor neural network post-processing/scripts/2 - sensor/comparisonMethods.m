close all
clear all

rs = pi;
Ds=0.0478;
Ls=0.0640;

load('C:/Users/debora/Downloads/data.mat');
load('C:/Users/debora/Downloads/dataValid.mat');
%% Defining the Problem
% The script *prprob* defines a matrix X with 26 columns, one for
% each letter of the alphabet.  Each column has 35 values which can either
% be 1 or 0.  Each column of 35 values defines a 5x7 bitmap of a letter.
%
% The matrix T is a 26x26 identity matrix which maps the 26 input
% vectors to the 26 classes.

load('data.mat');
in = sensor;
out = data(:,2:3);

load('dataVal.mat');
Xtest = sensor;
Tteste = data(:,2:3);
tTeste = data(:,1);


%% METODO C
setdemorandstream(rs);
net1 = feedforwardnet([50,100,100,200]);
net1.layers{1}.transferFcn='tansig';
net1.layers{2}.transferFcn='tansig';
net1.layers{3}.transferFcn='tansig';
net1.layers{4}.transferFcn='tansig';
net1.layers{5}.transferFcn='tansig';
net1.trainFcn='trainscg';
net1.divideParam.trainRatio = 0.6;
net1.divideParam.trainRatio = 0.25;
net1.divideParam.testRatio = 0.15;
net1 = train(net1,in',out',nnMATLAB);

Y1 = net1(Xtest');
%% METODO A
dOrtoA = zeros(size(Xtest,1),1);
thetaA = zeros(size(Xtest,1),1);
for i=1:size(Xtest,1)
    z1 = Xtest(i,1:7); 
    z2 = Xtest(i,8:14);
    
    [~,i1] = sort(z1);
    [~,i2] = sort(z2);
    
    pi1 = (i1(1) - 4)/3;
    pi2 = ((i2(1) + 7) - 11)/3;
    
    thetaA(i) = atan2((pi2 - pi1)*Ds,Ls);
    dOrtoA(i) = -(pi1 + pi2)*Ds/2;    
end
%% METODO B
dOrtoB = zeros(size(Xtest,1),1);
thetaB = zeros(size(Xtest,1),1);
for i=1:size(Xtest,1)
    z1 = Xtest(i,1:7); 
    z2 = Xtest(i,8:14);
    
    [~,i1] = sort(z1);
    [~,i2] = sort(z2);
    
    i1a = i1(1);
    i2a = i2(1);
    i1b = i1(2);
    i2b = i2(2);
    
    i1 = (i1a*(1-z1(i1a)) + i1b*(1-z1(i1b)))/((1-z1(i1a)) + (1-z1(i1b)));
    i2 = ((i2a+7)*(1-z2(i2a)) + (i2b+7)*(1-z2(i2b)))/((1-z2(i2a)) + (1-z2(i2b)));
    
    pi1 = (i1 - 4)/3;
    pi2 = (i2 - 11)/3;
    
    thetaB(i) = atan2((pi2 - pi1)*Ds,Ls);
    dOrtoB(i) = -(pi1 + pi2)*Ds/(2);    
end
%% Resultados
lineStyles = linspecer(4);

figure('Position', [10 10 600 400]);
subplot(2,1,1);
plot(tTeste,Tteste(:,1),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,thetaA,'-.','LineWidth',1.5,...
    'color',lineStyles(2,:));
legend('Real','Method A');
xlabel('time [s]');
ylabel('angle');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
subplot(2,1,2);
plot(tTeste,Tteste(:,2),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,dOrtoA,'-.','LineWidth',1.5,...
    'color',lineStyles(2,:));
legend('Real','Method A');
xlabel('time [s]');
ylabel('Orthogonal distance [m]');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

[~,~,~] = mkdir('pics');
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
saveas(gca,"pics/"+"methodA"+".png")

%%
figure('Position', [10 10 600 400]);
subplot(2,1,1);
plot(tTeste,Tteste(:,1),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,thetaB,'-.','LineWidth',1.5,...
    'color',lineStyles(3,:));
legend('Real','Method B');
xlabel('time [s]');
ylabel('angle');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
subplot(2,1,2);
plot(tTeste,Tteste(:,2),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,dOrtoB,'-.','LineWidth',1.5,...
    'color',lineStyles(3,:));
legend('Real','Method B');
xlabel('time [s]');
ylabel('Orthogonal distance [m]');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

[~,~,~] = mkdir('pics');
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
saveas(gca,"pics/"+"methodB"+".png")

%%
figure('Position', [10 10 600 400]);
subplot(2,1,1);
plot(tTeste,Tteste(:,1),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,Y1(1,:),'-.','LineWidth',1.5,...
    'color',lineStyles(4,:));
legend('Real','Method C');
xlabel('time [s]');
ylabel('angle');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
subplot(2,1,2);
plot(tTeste,Tteste(:,2),'LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on;
plot(tTeste,Y1(2,:),'-.','LineWidth',1.5,...
    'color',lineStyles(4,:));
legend('Real','Method C');
xlabel('time [s]');
ylabel('Orthogonal distance [m]');
grid on;
xlim([0 100]);
hold off
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

[~,~,~] = mkdir('pics');
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
saveas(gca,"pics/"+"methodC"+".png")
%%
immse(Tteste,Y1')
immse(Tteste,[thetaA';dOrtoA']')
immse(Tteste,[thetaB';dOrtoB']')