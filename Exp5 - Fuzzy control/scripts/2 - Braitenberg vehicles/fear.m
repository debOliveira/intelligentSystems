%% Clearing workspace
clear all
close all
clc
% %% global variables
fuzzyControllerCmdLine();
%% Loading coppeliaSim remote API - client side
sim=remApi('remoteApi');
%% Closing any previously opened connections
sim.simxFinish(-1);
%% Connecting to remote coppeliaSim API server
connectionAddress='127.0.0.1';
connectionPort=19997;
waitUntilConnected=true;
doNotReconnectOnceDisconnected=true;
timeOutInMs=5000;
commThreadCycleInMs=5;
res=0;
while(res == 0)
    [clientID]=sim.simxStart(connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs);
    if(clientID > -1)
        sim.simxSynchronous(clientID,true);
        fprintf('Starting simpleTest.m\n');
        res=1;
    else
        fprintf ('Waiting for coppeliaSim ...\n');
    end
end
%% Getting robot handle
e=1;
while (e~=0)
    [e,rob]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
    [e,leftMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
	[e,rightMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
    [e,rightSensorHandle]=sim.simxGetObjectHandle(clientID,'RightSensorP',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
    [e,leftSensorHandle]=sim.simxGetObjectHandle(clientID,'LeftSensorP',sim.simx_opmode_blocking);
end
fprintf('Got handles...\n');
%% Defining coppeliaSim client side parameters
np=2000;
hd=50e-3;
tf=np*hd;
tc=0;
td=0;
id=1;
%
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,3);
RS=zeros(np,1);
LS=zeros(np,1);
%% Initialization
Ds=0.0478;
D=195e-3;
Ls=0.0640;
R=D/2;
L=381e-3;
%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);
fprintf('Simulation started... Please restart if empty below this line\n');
%% Get initial pose = [position orientation]
e=1;
while (e~=0)
    [e,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,~,~,~]=sim.simxReadVisionSensor(clientID,rightSensorHandle,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,~,~,~]=sim.simxReadVisionSensor(clientID,leftSensorHandle,sim.simx_opmode_streaming);
end
%
fprintf('First values read\n');
%% Main control loop - coppeliaSim client side
while (td<tf)
    %% Current sampling instant
    t(id)=td;
    %% Measuring
    [~,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_buffer);
    [~,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_buffer);
    [~,~,rData,~]=sim.simxReadVisionSensor(clientID,rightSensorHandle,sim.simx_opmode_buffer);
    [~,~,lData,~]=sim.simxReadVisionSensor(clientID,leftSensorHandle,sim.simx_opmode_buffer);
    %% Calculating   
    result = evalfis(Fear,[lData(11) rData(11)]);
    leftVel =  result(1);
    rightVel = result(2);
    fprintf("%.2f %.2f\n",leftVel,rightVel);
    %% Actuating
    [~] = sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel, sim.simx_opmode_oneshot);
    [~] = sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel, sim.simx_opmode_oneshot);
    % Saving
    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id,:)=robOri(3); 
    RS(id)=rData(11);
    LS(id)=lData(11);
    %% Next sampling instant
    td=td+hd;
    id=id+1;
    sim.simxSynchronousTrigger(clientID);
end
%% Stoping coppeliaSim simulation
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait);
fprintf('Ending simpleTest.m\n');
sim.simxFinish(clientID);
sim.delete();
%% Plotting
lineStyles = linspecer(6);

figure('Position', [10 10 500 450]);

h(2) = scatter(0,0,500,"filled","o",...
        'MarkerEdgeColor',lineStyles(4,:),...
        'MarkerFaceColor',lineStyles(4,:),...
        'LineWidth',2.0); 
grid, hold on;               
viscircles([0 0],0.66,'LineStyle', '--','LineWidth',1.5,...
            'Color',lineStyles(5,:)); 
h(1) = plot(xp,yp,'-',...
            'color',lineStyles(1,:),'LineWidth',1.5);
for i=1:round(length(xp)/25):length(xp)
    drawRobot(xp(i),yp(i),fp(i),0.02,lineStyles(1,:));
end
h(3) = scatter(xp(1),yp(1),50,"filled","d",...
        'MarkerEdgeColor',lineStyles(2,:),...
        'MarkerFaceColor',lineStyles(2,:),...
        'LineWidth',2.0);    
h(4) = scatter(xp(end),yp(end),50,"filled","d",...
        'MarkerEdgeColor',lineStyles(3,:),...
        'MarkerFaceColor',lineStyles(3,:),...
        'LineWidth',2.0);     
grid on;
axis([-1.5 2.5 -1.5 2.5]);
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR}'},...
         'FontWeight','Normal');
legend(h([1 2 3 4]),"Path","Light point",...
                "Begin point","End Point",...
                'Location','southeast')
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
saveas(gca,"pics/"+"fear"+".png")