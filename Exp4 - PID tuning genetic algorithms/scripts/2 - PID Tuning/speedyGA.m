% SpeedyGA is a vectorized implementation of a Simple Genetic Algorithm in Matlab
% Version 1.3
% Copyright (C) 2007, 2008, 2009  Keki Burjorjee
% Created and tested under Matlab 7 (R14). 

%  Licensed under the Apache License, Version 2.0 (the "License"); you may
%  not use this file except in compliance with the License. You may obtain 
%  a copy of the License at  
%
%  http://www.apache.org/licenses/LICENSE-2.0 
%
%  Unless required by applicable law or agreed to in writing, software 
%  distributed under the License is distributed on an "AS IS" BASIS, 
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
%  See the License for the specific language governing permissions and 
%  limitations under the License. 

%  Acknowledgement of the author (Keki Burjorjee) is requested, but not required, 
%  in any publication that presents results obtained by using this script 

%  Without Sigma Scaling, Stochastic Universal Sampling, and the generation of mask 
%  repositories, SpeedyGA faithfully implements the specification of a simple genetic 
%  algorithm given on pages 10,11 of M. Mitchell's book An Introduction to
%  Genetic Algorithms, MIT Press, 1996). Selection is fitness
%  proportionate.

close all;
clear all;

global len popSize xini xg yg fg v0 pd tfin optionsODE

xg=2; yg=2; v0=0.25; pd =50;
fg=atan2(yg,xg);
tfin=20;
xini=[-1.5;-1.5;-fg;eps;eps];

optionsODE=odeset('RelTol',1e-6);

len=60;                    % The length of the genomes  
popSize=40;               % The size of the population (must be an even number)
maxGens=500;              % The maximum number of generations allowed in a run
probCrossover=1;           % The probability of crossing over. 
probMutation=0.003;        % The mutation probability (per bit)
sigmaScalingFlag=1;        % Sigma Scaling is described on pg 168 of M. Mitchell's
                           % GA book. It often improves GA performance.
sigmaScalingCoeff=1;       % Higher values => less fitness pressure 

SUSFlag=1;                 % 1 => Use Stochastic Universal Sampling (pg 168 of 
                           %      M. Mitchell's GA book)
                           % 0 => Do not use Stochastic Universal Sampling
                           %      Stochastic Universal Sampling almost always
                           %      improves performance

crossoverType=2;           % 0 => no crossover
                           % 1 => 1pt crossover
                           % 2 => uniform crossover

visualizationFlag=1;       % 0 => don't visualize bit frequencies
                           % 1 => visualize bit frequencies

verboseFlag=1;             % 1 => display details of each generation
                           % 0 => run quietly

useMaskRepositoriesFlag=1; % 1 => draw uniform crossover and mutation masks from 
                           %      a pregenerated repository of randomly generated bits. 
                           %      Significantly improves the speed of the code with
                           %      no apparent changes in the behavior of
                           %      the SGA
                           % 0 => generate uniform crossover and mutation
                           %      masks on the fly. Slower. 
                           
% crossover masks to use if crossoverType==0.
mutationOnlycrossmasks=false(popSize,len);

% pre-generate two “repositories” of random binary digits from which the  
% the masks used in mutation and uniform crossover will be picked. 
% maskReposFactor determines the size of these repositories.

maskReposFactor=5;
uniformCrossmaskRepos=rand(popSize/2,(len+1)*maskReposFactor)<0.5;
mutmaskRepos=rand(popSize,(len+1)*maskReposFactor)<probMutation;

% preallocate vectors for recording the average and maximum fitness in each
% generation
avgFitnessHist=zeros(1,maxGens+1);
maxFitnessHist=zeros(1,maxGens+1);

    
eliteIndiv=[];
eliteFitness=-realmax;


% the population is a popSize by len matrix of randomly generated boolean
% values
pop=rand(popSize,len)<.5;

for gen=0:maxGens

    % evaluate the fitness of the population. The vector of fitness values 
    % returned  must be of dimensions 1 x popSize.
    fitnessVals=fitting(pop);
     
    [maxFitnessHist(1,gen+1),maxIndex]=max(fitnessVals);    
    avgFitnessHist(1,gen+1)=mean(fitnessVals);
    if eliteFitness<maxFitnessHist(gen+1)
        eliteFitness=maxFitnessHist(gen+1);
        eliteIndiv=pop(maxIndex,:);
    end    
    
    % display the generation number, the average Fitness of the population,
    % and the maximum fitness of any individual in the population
    if verboseFlag
        display(['gen=' num2str(gen,'%.3d') '   avgFitness=' ...
            num2str(avgFitnessHist(1,gen+1),'%3.3f') '   maxFitness=' ...
            num2str(maxFitnessHist(1,gen+1),'%3.3f')]);
    end
    % Conditionally perform bit-frequency visualization
    if visualizationFlag
        figure(1)
        set (gcf, 'color', 'w');
        hold off
        bitFreqs=sum(pop)/popSize;
        plot(1:len,bitFreqs, '.');
        axis([0 len 0 1]);
        title(['Generation = ' num2str(gen) ', Average Fitness = ' sprintf('%0.3f', avgFitnessHist(1,gen+1))]);
        ylabel('Frequency of the Bit 1');
        xlabel('Locus');
        drawnow;
    end    
    
    if (any(isinf(fitnessVals)))
        break;
    end
    
    % Conditionally perform sigma scaling 
    if sigmaScalingFlag
        sigma=std(fitnessVals);
        if sigma~=0;
            fitnessVals=1+(fitnessVals-mean(fitnessVals))/...
            (sigmaScalingCoeff*sigma);
            fitnessVals(fitnessVals<=0)=0;
        else
            fitnessVals=ones(popSize,1);
        end
    end        
    
    
    % Normalize the fitness values and then create an array with the 
    % cumulative normalized fitness values (the last value in this array
    % will be 1)
    cumNormFitnessVals=cumsum(fitnessVals/sum(fitnessVals));

    % Use fitness proportional selection with Stochastic Universal or Roulette
    % Wheel Sampling to determine the indices of the parents 
    % of all crossover operations
    if SUSFlag
        markers=rand(1,1)+[1:popSize]/popSize;
        markers(markers>1)=markers(markers>1)-1;
    else
        markers=rand(1,popSize);
    end
    [temp parentIndices]=histc(markers,[0 cumNormFitnessVals]);
    parentIndices=parentIndices(randperm(popSize));    

    % deterimine the first parents of each mating pair
    firstParents=pop(parentIndices(1:popSize/2),:);
    % determine the second parents of each mating pair
    secondParents=pop(parentIndices(popSize/2+1:end),:);
    
    % create crossover masks
    if crossoverType==0
        masks=mutationOnlycrossmasks;
    elseif crossoverType==1
        masks=false(popSize/2, len);
        temp=ceil(rand(popSize/2,1)*(len-1));
        for i=1:popSize/2
            masks(i,1:temp(i))=true;
        end
    else
        if useMaskRepositoriesFlag
            temp=floor(rand*len*(maskReposFactor-1));
            masks=uniformCrossmaskRepos(:,temp+1:temp+len);
        else
            masks=rand(popSize/2, len)<.5;
        end
    end
    
    % determine which parent pairs to leave uncrossed
    reprodIndices=rand(popSize/2,1)<1-probCrossover;
    masks(reprodIndices,:)=false;
    
    % implement crossover
    firstKids=firstParents;
    firstKids(masks)=secondParents(masks);
    secondKids=secondParents;
    secondKids(masks)=firstParents(masks);
    pop=[firstKids; secondKids];
    
    % implement mutation
    if useMaskRepositoriesFlag
        temp=floor(rand*len*(maskReposFactor-1));
        masks=mutmaskRepos(:,temp+1:temp+len);
    else
        masks=rand(popSize, len)<probMutation;
    end
    pop=xor(pop,masks);    
end
if verboseFlag
    figure(2)
    %set(gcf,'Color','w');
    hold off
    plot([0:maxGens],avgFitnessHist,'k-');
    hold 
    plot([0:maxGens],maxFitnessHist,'c-');
    title('Maximum and Average Fitness')
    xlabel('Generation')
    ylabel('Fitness')
end

%%
kp=0.1; ki=0.1; kd=0.01;
%[x1,t1,x2,t2] = gotogoalPID(kp,ki,kd);
[t1,x1]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
[kp,ki,kd] = binArray2dec(eliteIndiv);
fprintf("kp = "+num2str(kp)+" ki = "+num2str(ki)+" kd = "+num2str(kd)+"\n");
[t0,x0]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
%%
figure('Position', [0 0 600 400])
lineStyles = linspecer(2);
h(1)=plot(x0(:,1),x0(:,2),'b-','LineWidth',2.0,...
    'color',lineStyles(1,:));
xlabel('x, (m)'), ylabel('y, (m)')
hold on
for i=1:round(length(x0(:,1))/30):length(x0(:,1))
    desenherobo(x0(i,1),x0(i,2),x0(i,3),0.03,lineStyles(1,:));
end
h(2)=plot(x1(:,1),x1(:,2),'g-','LineWidth',2.0,...
    'color',lineStyles(2,:));
xlabel('x, (m)'), ylabel('y, (m)')
hold on
for i=1:round(length(x1(:,1))/30):length(x1(:,1))
    desenherobo(x1(i,1),x1(i,2),x1(i,3),0.03,lineStyles(2,:));
end
axis equal
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR with PID control law}'; ...
        ['{\it\fontsize{10} Cost function = ITAE}', ......
        '{\it\fontsize{10}   k_p} = ',num2str(kp) ...
         '{\it\fontsize{10}   k_i} = ',num2str(ki) ...
         '{\it\fontsize{10}   k_d} = ',num2str(kd) ...
         '{\it\fontsize{10}   p_d} = ',num2str(pd)]},...
         'FontWeight','Normal');
legend(h([2 1]),"Original gains",...
    "Genetic algorithm",'Location','southeast')
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
nameFile="ITAEnewOrigin";
saveas(gca,"pics/"+nameFile+".png")

[~,~,~] = mkdir('data');
save("data/"+nameFile+".mat",'eliteIndiv')
%%
function fitness=fitting(pop)
    [kp,ki,kd]=binArray2dec(pop);
    
    global xg yg popSize xini pd tfin optionsODE
    
    vobj = zeros(popSize,1);
    for i=1:popSize    
        [t,x]=ode45(@(t,x)smf(t,x,kp(i),ki(i),kd(i),pd),[0 tfin],xini,optionsODE);

        xp=x(:,1);
        yp=x(:,2);
        fp=x(:,3);

        fd=atan2(yg-yp,xg-xp);
        e=fd-fp;
        e=atan2(sin(e),cos(e));

        %IAE=trapz(t,abs(e));
        %ISE=trapz(t,e.^2);
        ITAE=trapz(t,t.*abs(e));
        %ITSE=trapz(t,t.*(e.^2));
        vobj(i) = ITAE;
    end
    fitness=(1./vobj)';
end
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
%%
function [x0,t0,x1,t1]=gotogoalPID(kp,ki,kd)
    %fmincon
    oldopts=optimset(@fmincon);
    optionsFMINCON=optimset(oldopts,'display','iter','TolCon',1e-9);
    %
    global pd xg yg v0 tfin xini optionsODE pd
    fg=atan2(yg,xg);
    %
    [t0,x0]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
    %
    fun=@fobj;
    A=[-1,0,0;0,-1,0;0,0,-1];b=[0;0;0];
    Aeq=[];beq=[];
    lb=[0,0,0];ub=[1,1,1];
    nonlcon=@mycon;
    nvars=3;
    %
    PID=fmincon(@fobj,[kp ki kd],A,b,Aeq,beq,lb,ub,nonlcon,optionsFMINCON);

    Kp=PID(1); Ki=PID(2); Kd=PID(3);
    %
    [t1,x1]=ode45(@(t,x)smf(t,x,Kp,Ki,Kd,pd),[0 tfin],xini,optionsODE);
end

function [vobj] = fobj(k)
    P=k(1);
    I=k(2);
    D=k(3);
    global tfin xini optionsODE pd yg xg
    %
    [t,x]=ode45(@(t,x)smf(t,x,P,I,D,pd),[0 tfin],xini,optionsODE);
    %
    xp=x(:,1);
    yp=x(:,2);
    fp=x(:,3);
    %
    %e=sqrt((xg-xp).^2+(yg-yp).^2);
    %
    fd=atan2(yg-yp,xg-xp);
    e=fd-fp;
    e=atan2(sin(e),cos(e));
    %
    %% Performance criteria
    %IAE=trapz(t,abs(e));
    %ISE=trapz(t,e.^2);
    %ITAE=trapz(t,t.*abs(e));
    ITSE=trapz(t,t.*(e.^2));
    %
    vobj=ITSE;
end

function [c,ceq] = mycon(x)
    global pd
    gp=x(1);gi=x(2);gd=x(3);
    %
    c(1) = -(gi+gp*pd)/(gp+gd*pd);
    c(2) = -(gi*pd)/(gp+gd*pd);
    c(3) = -(gi*(pd+1)+gp*pd)/(gp+gd*pd);
    %
    ceq = [];
end