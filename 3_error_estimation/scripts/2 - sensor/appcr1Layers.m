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

%% Training the Neural Network
% We would like the network to not only recognize perfectly formed
% letters, but also noisy versions of the letters.  So we will try training
% a second network on noisy data and compare its ability to genearlize
% with the first network.
%
% Here 30 noisy copies of each letter Xn are created.  Values are limited
% by *min* and *max* to fall between 0 and 1.  The corresponding targets
% Tn are also defined.

% setdemorandstream(rs);
% numNoise = 30;
% Xn = min(max(repmat(X,1,numNoise)+randn(35,36*numNoise)*0.2,0),1);
% Tn = repmat(T,1,numNoise);

%%
% Here the first network is created and trained.

setdemorandstream(rs);

net1 = feedforwardnet(25);
net1.layers{1}.transferFcn='tansig';
net1.layers{2}.transferFcn='tansig';
net1.trainFcn='trainscg';

net1 = train(net1,in',out',nnMATLAB);

%%
% Here the second network is created and trained.

setdemorandstream(rs);

net2 = feedforwardnet([25,25]);
net2.layers{1}.transferFcn='tansig';
net2.layers{2}.transferFcn='tansig';
net2.layers{3}.transferFcn='tansig';
net2.trainFcn='trainscg';

net2 = train(net2,in',out',nnMATLAB);

%%
% Here the third network is created and trained.

setdemorandstream(rs);

net3 = feedforwardnet([25,25,25]);
net3.layers{1}.transferFcn='tansig';
net3.layers{2}.transferFcn='tansig';
net3.layers{3}.transferFcn='tansig';
net3.layers{4}.transferFcn='tansig';
net3.trainFcn='trainscg';

net3 = train(net3,in',out',nnMATLAB);

%%
% Here the third network is created and trained.

setdemorandstream(rs);

net4 = feedforwardnet([25,25,25,25]);
net4.layers{1}.transferFcn='tansig';
net4.layers{2}.transferFcn='tansig';
net4.layers{3}.transferFcn='tansig';
net4.layers{4}.transferFcn='tansig';
net4.layers{5}.transferFcn='tansig';
net4.trainFcn='trainscg';

net4 = train(net4,in',out',nnMATLAB);

%%
% Here the third network is created and trained.

setdemorandstream(rs);

net5 = feedforwardnet([25,25,25,25,25]);
net5.layers{1}.transferFcn='tansig';
net5.layers{2}.transferFcn='tansig';
net5.layers{3}.transferFcn='tansig';
net5.layers{4}.transferFcn='tansig';
net5.layers{5}.transferFcn='tansig';
net5.layers{5}.transferFcn='tansig';
net5.trainFcn='trainscg';

net5 = train(net5,in',out',nnMATLAB);
%% Testing Both Neural Networks
Xtest = sensor;
Tteste = data(:,2:3);

Y1 = net1(Xtest');
Y2 = net2(Xtest');
Y3 = net3(Xtest');
Y4 = net4(Xtest');
Y5 = net5(Xtest');
%%
lineStyles = linspecer(5);

figure('Position', [10 10 500 350]);
C = categorical({'1','2','3','4','5'});
Y = [perform(net1,Tteste',Y1),...
    perform(net2,Tteste',Y2),...
    perform(net3,Tteste',Y3),...
    perform(net4,Tteste',Y4),...
    perform(net5,Tteste',Y5)];
b = bar(C,Y,...
    'BarWidth', 0.2);
b.FaceColor = 'flat';
b.CData(1,:) = lineStyles(1,:);
b.CData(2,:) = lineStyles(2,:);
b.CData(3,:) = lineStyles(3,:);
b.CData(4,:) = lineStyles(4,:);
b.CData(5,:) = lineStyles(5,:);
text(1:length(Y),Y,num2str(Y','%.6f'),...
    'vert','bottom','horiz','center',...
    'Fontsize',10);
xlabel('Number of hidden layers');
ylabel('Mean-squared error');
title('Performance of Neural Networks');
ylim([0 0.003]);
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
saveas(gca,"pics/"+name+"1.png")
%%
% Network 1, trained without noise, has more errors due to noise than
% does Network 2, which was trained with noise.