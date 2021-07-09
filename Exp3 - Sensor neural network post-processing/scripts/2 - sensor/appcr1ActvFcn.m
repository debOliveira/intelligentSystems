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

load('data.mat');

in = sensor;
out = data(:,2:3);


load('dataVal.mat');

%%
setdemorandstream(rs);
net1 = feedforwardnet([50,100,100,200]);
net1.layers{1}.transferFcn='purelin';
net1.layers{2}.transferFcn='purelin';
net1.layers{3}.transferFcn='purelin';
net1.layers{4}.transferFcn='purelin';
net1.layers{5}.transferFcn='purelin';
net1.trainFcn='trainscg';
net1.divideParam.trainRatio = 0.6;
net1.divideParam.trainRatio = 0.25;
net1.divideParam.testRatio = 0.15;
net1 = train(net1,in',out',nnMATLAB);
%%
setdemorandstream(rs);
net2 = feedforwardnet([50,100,100,200]);
net2.layers{1}.transferFcn='tansig';
net2.layers{2}.transferFcn='tansig';
net2.layers{3}.transferFcn='tansig';
net2.layers{4}.transferFcn='tansig';
net2.layers{5}.transferFcn='tansig';
net2.trainFcn='trainscg';
net2.divideParam.trainRatio = 0.6;
net2.divideParam.trainRatio = 0.25;
net2.divideParam.testRatio = 0.15;
net2 = train(net2,in',out',nnMATLAB);
%%
setdemorandstream(rs);
net3 = feedforwardnet([50,100,100,200]);
net3.layers{1}.transferFcn='logsig';
net3.layers{2}.transferFcn='logsig';
net3.layers{3}.transferFcn='logsig';
net3.layers{4}.transferFcn='logsig';
net3.layers{5}.transferFcn='logsig';
net3.trainFcn='trainscg';
net3.divideParam.trainRatio = 0.6;
net3.divideParam.trainRatio = 0.25;
net3.divideParam.testRatio = 0.15;
net3 = train(net3,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net4 = feedforwardnet([50,100,100,200]);
net4.layers{1}.transferFcn='tansig';
net4.layers{2}.transferFcn='tansig';
net4.layers{3}.transferFcn='tansig';
net4.layers{4}.transferFcn='tansig';
net4.layers{5}.transferFcn='purelin';
net4.trainFcn='trainscg';
net4.divideParam.trainRatio = 0.6;
net4.divideParam.trainRatio = 0.25;
net4.divideParam.testRatio = 0.15;
net4 = train(net4,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net5 = feedforwardnet([50,100,100,200]);
net5.layers{1}.transferFcn='logsig';
net5.layers{2}.transferFcn='logsig';
net5.layers{3}.transferFcn='logsig';
net5.layers{4}.transferFcn='logsig';
net5.layers{5}.transferFcn='purelin';
net5.trainFcn='trainscg';
net5.divideParam.trainRatio = 0.6;
net5.divideParam.trainRatio = 0.25;
net5.divideParam.testRatio = 0.15;
net5 = train(net5,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net6 = feedforwardnet([50,100,100,200]);
net6.layers{1}.transferFcn='purelin';
net6.layers{2}.transferFcn='purelin';
net6.layers{3}.transferFcn='purelin';
net6.layers{4}.transferFcn='purelin';
net6.layers{5}.transferFcn='tansig';
net6.divideParam.trainRatio = 0.6;
net6.divideParam.trainRatio = 0.25;
net6.divideParam.testRatio = 0.15;
net6.trainFcn='trainscg';
net6 = train(net6,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net7 = feedforwardnet([50,100,100,200]);
net7.layers{1}.transferFcn='logsig';
net7.layers{2}.transferFcn='logsig';
net7.layers{3}.transferFcn='logsig';
net7.layers{4}.transferFcn='logsig';
net7.layers{5}.transferFcn='tansig';
net7.trainFcn='trainscg';
net7.divideParam.trainRatio = 0.6;
net7.divideParam.trainRatio = 0.25;
net7.divideParam.testRatio = 0.15;
net7 = train(net7,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net8 = feedforwardnet([50,100,100,200]);
net8.layers{1}.transferFcn='purelin';
net8.layers{2}.transferFcn='purelin';
net8.layers{3}.transferFcn='purelin';
net8.layers{4}.transferFcn='purelin';
net8.layers{5}.transferFcn='logsig';
net8.divideParam.trainRatio = 0.6;
net8.divideParam.trainRatio = 0.25;
net8.divideParam.testRatio = 0.15;
net8.trainFcn='trainscg';
net8 = train(net8,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net9 = feedforwardnet([50,100,100,200]);
net9.layers{1}.transferFcn='tansig';
net9.layers{2}.transferFcn='tansig';
net9.layers{3}.transferFcn='tansig';
net9.layers{4}.transferFcn='tansig';
net9.layers{5}.transferFcn='logsig';
net9.divideParam.trainRatio = 0.6;
net9.divideParam.trainRatio = 0.25;
net9.divideParam.testRatio = 0.15;
net9.trainFcn='trainscg';
net9 = train(net9,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net10 = feedforwardnet([50,100,100,200]);
net10.layers{1}.transferFcn='purelin';
net10.layers{2}.transferFcn='purelin';
net10.layers{3}.transferFcn='purelin';
net10.layers{4}.transferFcn='purelin';
net10.layers{5}.transferFcn='softmax';
net10.divideParam.trainRatio = 0.6;
net10.divideParam.trainRatio = 0.25;
net10.divideParam.testRatio = 0.15;
net10.trainFcn='trainscg';
net10 = train(net10,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net11 = feedforwardnet([50,100,100,200]);
net11.layers{1}.transferFcn='tansig';
net11.layers{2}.transferFcn='tansig';
net11.layers{3}.transferFcn='tansig';
net11.layers{4}.transferFcn='tansig';
net11.layers{5}.transferFcn='softmax';
net11.trainFcn='trainscg';
net11.divideParam.trainRatio = 0.6;
net11.divideParam.trainRatio = 0.25;
net11.divideParam.testRatio = 0.15;
net11 = train(net11,in',out',nnMATLAB);
%% 
setdemorandstream(rs);
net12 = feedforwardnet([50,100,100,200]);
net12.layers{1}.transferFcn='logsig';
net12.layers{2}.transferFcn='logsig';
net12.layers{3}.transferFcn='logsig';
net12.layers{4}.transferFcn='logsig';
net12.layers{5}.transferFcn='softmax';
net12.trainFcn='trainscg';
net12.divideParam.trainRatio = 0.6;
net12.divideParam.trainRatio = 0.25;
net12.divideParam.testRatio = 0.15;
net12 = train(net12,in',out',nnMATLAB);
%% Testing Both Neural Networks
Xtest = sensor;
Tteste = data(:,2:3);

Y1 = net1(Xtest');
Y2 = net2(Xtest');
Y3 = net3(Xtest');
Y4 = net4(Xtest');
Y5 = net5(Xtest');
Y6 = net6(Xtest');
Y7 = net7(Xtest');
Y8 = net8(Xtest');
Y9 = net9(Xtest');
Y10 = net10(Xtest');
Y11 = net11(Xtest');
Y12 = net12(Xtest');
%%
lineStyles = linspecer(12);

figure('Position', [10 10 900 400]);
C = categorical({'purelin','tansig','logsig', ...
       'tansig-purelin','logsig-purelin',...
       'purelin-tansig','logsig-tansig',...
       'purelin-logsig','tansig-logsig',...
       'purelin-softmax','tansig-softmax','logsig-softmax'});
C = reordercats(C,{'purelin','tansig','logsig', ...
       'tansig-purelin','logsig-purelin',...
       'purelin-tansig','logsig-tansig',...
       'purelin-logsig','tansig-logsig',...
       'purelin-softmax','tansig-softmax','logsig-softmax'});
Y = [perform(net1,Tteste',Y1),...
    perform(net2,Tteste',Y2),...
    perform(net3,Tteste',Y3),...
    perform(net4,Tteste',Y4),...
    perform(net5,Tteste',Y5),...
    perform(net6,Tteste',Y6),...
    perform(net7,Tteste',Y7),...
    perform(net8,Tteste',Y8),...
    perform(net9,Tteste',Y9),...
    perform(net10,Tteste',Y10),...
    perform(net11,Tteste',Y11),...
    perform(net12,Tteste',Y12)];
b = bar(C,Y,...
    'BarWidth', 0.2);
b.FaceColor = 'flat';
b.CData(1,:) = lineStyles(1,:);
b.CData(2,:) = lineStyles(2,:);
b.CData(3,:) = lineStyles(3,:);
b.CData(4,:) = lineStyles(4,:);
b.CData(5,:) = lineStyles(5,:);
b.CData(6,:) = lineStyles(6,:);
b.CData(7,:) = lineStyles(7,:);
b.CData(8,:) = lineStyles(8,:);
b.CData(9,:) = lineStyles(9,:);
b.CData(10,:) = lineStyles(10,:);
b.CData(11,:) = lineStyles(11,:);
b.CData(12,:) = lineStyles(12,:);
text(1:length(Y),Y,num2str(Y','%.6f'),...
    'vert','bottom','horiz','center',...
    'Fontsize',10);
xlabel('Activation functions');
ylabel('Mean-squared error');
title('Performance of Neural Networks');
ylim([0 0.70]);
grid on;
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