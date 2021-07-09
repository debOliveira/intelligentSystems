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

numNoise = 30;
Xn = min(max(repmat(X,1,numNoise)+randn(35,36*numNoise)*0.2,0),1);
Tn = repmat(T,1,numNoise);

%%
% Here the first network is created and trained.

setdemorandstream(rs);

net1 = feedforwardnet(25);
net1.layers{1}.transferFcn='tansig';
net1.layers{2}.transferFcn='tansig';
net1.trainFcn='trainscg';

net1 = train(net1,Xn,Tn,nnMATLAB);

%%
% Here the second network is created and trained.

setdemorandstream(rs);

net2 = feedforwardnet([25,25]);
net2.layers{1}.transferFcn='tansig';
net2.layers{2}.transferFcn='tansig';
net2.trainFcn='trainscg';

net2 = train(net2,Xn,Tn,nnMATLAB);

%%
% Here the third network is created and trained.

setdemorandstream(rs);

net3 = feedforwardnet([25,25,25]);
net3.layers{1}.transferFcn='tansig';
net3.layers{2}.transferFcn='tansig';
net3.trainFcn='trainscg';

net3 = train(net3,Xn,Tn,nnMATLAB);
%% Testing Both Neural Networks

lineStyles = linspecer(3);

noiseLevels = 0:.05:1;
numLevels = length(noiseLevels);
percError1 = zeros(1,numLevels);
percError2 = zeros(1,numLevels);
percError3 = zeros(1,numLevels);
for i = 1:numLevels
    
  Xtest = min(max(repmat(X,1,numNoise)+randn(35,36*numNoise)*noiseLevels(i),0),1);
  Y1 = net1(Xtest);
  percError1(i) = sum(sum(abs(Tn-compet(Y1))))/(36*numNoise*2);
  Y2 = net2(Xtest);
  percError2(i) = sum(sum(abs(Tn-compet(Y2))))/(36*numNoise*2);
  Y3 = net3(Xtest);
  percError3(i) = sum(sum(abs(Tn-compet(Y3))))/(36*numNoise*2);
end

figure('Position', [10 10 500 350]);
plot(noiseLevels,percError1*100,'-','LineWidth',1.5,...
    'color',lineStyles(1,:)); hold on,
plot(noiseLevels,percError2*100,'-','LineWidth',1.5,...
    'color',lineStyles(2,:));
plot(noiseLevels,percError3*100,'-','LineWidth',1.5,...
    'color',lineStyles(3,:));
title('Percentage of Recognition Errors');
grid on;
xlabel('Noise Level');
ylabel('Errors');
legend('2 hidden layers','3 hidden layers','4 hidden layers','Location','SouthEast')

ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

p = mfilename('fullpath')

saveas(gca,filename)
%%
% Network 1, trained without noise, has more errors due to noise than
% does Network 2, which was trained with noise.