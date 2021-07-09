%%
clc
clear all

%% Time span

tStart = 0;
tEnd = 20;
timeSpan = [tStart tEnd];
%% Initial conditions

initPhi = pi/4+pi;
initPos = [-1.5;-1.5];
initR = 0;
z=[initPos;initPhi;initR];
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
    [clientID]=sim.simxStart(connectionAddress,connectionPort,...
        waitUntilConnected,doNotReconnectOnceDisconnected,...
        timeOutInMs,commThreadCycleInMs);
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
    [e,rob]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',...
        sim.simx_opmode_blocking);
end
%% Defining coppeliaSim client side parameters
np=100*tEnd/5;
hd=50e-3;
tf=np*hd;
tc=0;
td=0;
id=1;
%
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,...
    sim.simx_opmode_oneshot_wait);
%% Get initial pose = [position orientation]
e=1;
while (e~=0)
    [e,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,...
        sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,...
        sim.simx_opmode_streaming);
end
%
%% Main control loop - coppeliaSim client side
while (td<tf)
    %% Current sampling instant
    t(id)=td;
    %% Measuring
    [~,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,...
        sim.simx_opmode_buffer);
    [~,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,...
        sim.simx_opmode_buffer);
    %% Saving robot pose - rob
    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id)=robOri(1,3);
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
%% Plotting results
[~,~,~] = mkdir('data');
save('data/ITAE.mat','xp','yp','fp');