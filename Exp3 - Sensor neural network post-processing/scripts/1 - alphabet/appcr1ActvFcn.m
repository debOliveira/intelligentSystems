%% Character Recognition
% This example illustrates how to train a neural network to perform
% simple character recognition.

% Copyright 2011-2012 The MathWorks, Inc.
close all
clear all

rs = pi;

%% Defining the Problem
% The script *prprob* defines a matrix X with 26 columns, one for
% each letter of the alphabet.  Each column has 35 values which can either
% be 1 or 0.  Each column of 35 values defines a 5x7 bitmap of a letter.
%
% The matrix T is a 26x26 identity matrix which maps the 26 input
% vectors to the 26 classes.

[X,T] = prprobWithNumbers;

%% Training the Neural Network
% We would like the network to not only recognize perfectly formed
% letters, but also noisy versions of the letters.  So we will try training
% a second network on noisy data and compare its ability to genearlize
% with the first network.
%
% Here 30 noisy copies of each letter Xn are created.  Values are limited
% by *min* and *max* to fall between 0 and 1.  The corresponding targets
% Tn are also defined.
setdemorandstream(rs);
numNoise = 30;
Xn = min(max(repmat(X,1,numNoise)+randn(35,36*numNoise)*0.2,0),1);
Tn = repmat(T,1,numNoise);

%%
setdemorandstream(rs);
net1 = feedforwardnet([80,50,25]);
net1.layers{1}.transferFcn='purelin';
net1.layers{2}.transferFcn='purelin';
net1.layers{3}.transferFcn='purelin';
net1.layers{4}.transferFcn='purelin';
net1.trainFcn='trainscg';
net1 = train(net1,Xn,Tn,nnMATLAB);
%%
setdemorandstream(rs);
net2 = feedforwardnet([80,50,25]);
net2.layers{1}.transferFcn='tansig';
net2.layers{2}.transferFcn='tansig';
net2.layers{3}.transferFcn='tansig';
net2.layers{4}.transferFcn='tansig';
net2.trainFcn='trainscg';
net2 = train(net2,Xn,Tn,nnMATLAB);
%%
setdemorandstream(rs);
net3 = feedforwardnet([80,50,25]);
net3.layers{1}.transferFcn='logsig';
net3.layers{2}.transferFcn='logsig';
net3.layers{3}.transferFcn='logsig';
net3.layers{4}.transferFcn='logsig';
net3.trainFcn='trainscg';
net3 = train(net3,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net4 = feedforwardnet([80,50,25]);
net4.layers{1}.transferFcn='tansig';
net4.layers{2}.transferFcn='tansig';
net4.layers{3}.transferFcn='tansig';
net4.layers{4}.transferFcn='purelin';
net4.trainFcn='trainscg';
net4 = train(net4,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net5 = feedforwardnet([80,50,25]);
net5.layers{1}.transferFcn='logsig';
net5.layers{2}.transferFcn='logsig';
net5.layers{3}.transferFcn='logsig';
net5.layers{4}.transferFcn='purelin';
net5.trainFcn='trainscg';
net5 = train(net5,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net6 = feedforwardnet([80,50,25]);
net6.layers{1}.transferFcn='purelin';
net6.layers{2}.transferFcn='purelin';
net6.layers{3}.transferFcn='purelin';
net6.layers{4}.transferFcn='tansig';
net6.trainFcn='trainscg';
net6 = train(net6,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net7 = feedforwardnet([80,50,25]);
net7.layers{1}.transferFcn='logsig';
net7.layers{2}.transferFcn='logsig';
net7.layers{3}.transferFcn='logsig';
net7.layers{4}.transferFcn='tansig';
net7.trainFcn='trainscg';
net7 = train(net7,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net8 = feedforwardnet([80,50,25]);
net8.layers{1}.transferFcn='purelin';
net8.layers{2}.transferFcn='purelin';
net8.layers{3}.transferFcn='purelin';
net8.layers{4}.transferFcn='logsig';
net8.trainFcn='trainscg';
net8 = train(net8,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net9 = feedforwardnet([80,50,25]);
net9.layers{1}.transferFcn='tansig';
net9.layers{2}.transferFcn='tansig';
net9.layers{3}.transferFcn='tansig';
net9.layers{4}.transferFcn='logsig';
net9.trainFcn='trainscg';
net9 = train(net9,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net10 = feedforwardnet([80,50,25]);
net10.layers{1}.transferFcn='purelin';
net10.layers{2}.transferFcn='purelin';
net10.layers{3}.transferFcn='purelin';
net10.layers{4}.transferFcn='softmax';
net10.trainFcn='trainscg';
net10 = train(net10,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net11 = feedforwardnet([80,50,25]);
net11.layers{1}.transferFcn='tansig';
net11.layers{2}.transferFcn='tansig';
net11.layers{3}.transferFcn='tansig';
net11.layers{4}.transferFcn='softmax';
net11.trainFcn='trainscg';
net11 = train(net11,Xn,Tn,nnMATLAB);
%% 
setdemorandstream(rs);
net12 = feedforwardnet([80,50,25]);
net12.layers{1}.transferFcn='logsig';
net12.layers{2}.transferFcn='logsig';
net12.layers{3}.transferFcn='logsig';
net12.layers{4}.transferFcn='softmax';
net12.trainFcn='trainscg';
net12 = train(net12,Xn,Tn,nnMATLAB);
%% Testing Both Neural Networks

lineStyles = linspecer(12,'sequential');
noiseLevels = 0:.05:1;
numLevels = length(noiseLevels);
percError1 = zeros(1,numLevels);
percError2 = zeros(1,numLevels);
percError3 = zeros(1,numLevels);
percError4 = zeros(1,numLevels);
percError5 = zeros(1,numLevels);
percError6 = zeros(1,numLevels);
percError7 = zeros(1,numLevels);
percError8 = zeros(1,numLevels);
percError9 = zeros(1,numLevels);
percError10 = zeros(1,numLevels);
percError11 = zeros(1,numLevels);
percError12 = zeros(1,numLevels);
for i = 1:numLevels    
  Xtest = min(max(repmat(X,1,numNoise)+randn(35,36*numNoise)*noiseLevels(i),0),1);
  Y1 = net1(Xtest);
  percError1(i) = sum(sum(abs(Tn-compet(Y1))))/(36*numNoise*2);
  Y2 = net2(Xtest);
  percError2(i) = sum(sum(abs(Tn-compet(Y2))))/(36*numNoise*2);
  Y3 = net3(Xtest);
  percError3(i) = sum(sum(abs(Tn-compet(Y3))))/(36*numNoise*2);
  Y4 = net4(Xtest);
  percError4(i) = sum(sum(abs(Tn-compet(Y4))))/(36*numNoise*2);
  Y5 = net5(Xtest);
  percError5(i) = sum(sum(abs(Tn-compet(Y5))))/(36*numNoise*2);
  Y6 = net6(Xtest);
  percError6(i) = sum(sum(abs(Tn-compet(Y6))))/(36*numNoise*2);
  Y7 = net7(Xtest);
  percError7(i) = sum(sum(abs(Tn-compet(Y7))))/(36*numNoise*2);
  Y8 = net8(Xtest);
  percError8(i) = sum(sum(abs(Tn-compet(Y8))))/(36*numNoise*2);
  Y9 = net9(Xtest);
  percError9(i) = sum(sum(abs(Tn-compet(Y9))))/(36*numNoise*2);
  Y10 = net10(Xtest);
  percError10(i) = sum(sum(abs(Tn-compet(Y10))))/(36*numNoise*2);
  Y11 = net11(Xtest);
  percError11(i) = sum(sum(abs(Tn-compet(Y11))))/(36*numNoise*2);
  Y12 = net12(Xtest);
  percError12(i) = sum(sum(abs(Tn-compet(Y12))))/(36*numNoise*2);
end

figure('Position', [10 10 900 400]);
plot(noiseLevels,percError1*100,'-.','LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on,
plot(noiseLevels,percError2*100,'-x','LineWidth',1.5,...
    'color',lineStyles(2,:));
plot(noiseLevels,percError3*100,'-o','LineWidth',1.5,...
    'color',lineStyles(3,:));
plot(noiseLevels,percError4*100,'-','LineWidth',1.5,...
    'color',lineStyles(4,:));
plot(noiseLevels,percError5*100,'--+','LineWidth',1.5,...
    'color',lineStyles(5,:).*[0.8 0.8 0.8]);
plot(noiseLevels,percError6*100,'-.','LineWidth',1.5,...
    'color',lineStyles(6,:).*[0.8 0.8 1]);
plot(noiseLevels,percError7*100,'-','LineWidth',1.5,...
    'color',lineStyles(7,:));
plot(noiseLevels,percError8*100,'-*','LineWidth',1.5,...
    'color',lineStyles(8,:));
plot(noiseLevels,percError9*100,'-','LineWidth',1.5,...
    'color',lineStyles(9,:));
plot(noiseLevels,percError10*100,':','LineWidth',2,...
    'color',lineStyles(10,:));
plot(noiseLevels,percError11*100,':','LineWidth',1.5,...
    'color',lineStyles(11,:));
plot(noiseLevels,percError12*100,'-.*','LineWidth',1.5,...
    'color',lineStyles(12,:));
title('Percentage of Recognition Errors');
grid on;
xlabel('Noise Level');
ylabel('Errors');
legend('purelin','tansig','logsig', ...
       'tansig-purelin','logsig-purelin',...
       'purelin-tansig','logsig-tansig',...
       'purelin-logsig','tansig-logsig',...
       'purelin-softmax','tansig-softmax','logsig-softmax',...
       'location','eastoutside')

%% Saving plot
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
saveas(gca,"pics/"+name+".png")
%%
% Network 1, trained without noise, has more errors due to noise than
% does Network 2, which was trained with noise.