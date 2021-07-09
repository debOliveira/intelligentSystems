close all; clc; clear all;
%% Time span

tStart = 0;
tEnd = 30;
timeSpan = [tStart tEnd];
%% Defining variables

global Kp v0 goal dMin

Kp = 0.5;
v0 = 0.2;
goal = [2;2];
dMin = 0.005;
%% Initial conditions

initPhi = pi/4+pi;
initPos = [-1.5;-1.5];
z=[initPos;initPhi];
%% Solving ODE
[timeSim,out] = ode45(@dinModel,timeSpan,z);
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
%%
figure();
h(1) = plot(out(:,1),out(:,2),'b--','LineWidth',2); grid, hold on;
axis equal
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR with P control law}'; ...
        ['{\it\fontsize{10} K_p} = ',num2str(Kp)]},...
         'FontWeight','Normal');
for i=1:round(size(out,1)/20):size(out,1)
    drawRobot(out(i,1),out(i,2),out(i,3),0.02,'b-');
end
h(3) = scatter(initPos(1),initPos(2),'filled',"dc");
h(4) = scatter(goal(1),goal(2),'filled',"or");
h(2) = plot(xp,yp,'r:','LineWidth',2.0);
for i=1:round(length(xp)/20):length(xp)
    drawRobot(xp(i),yp(i),fp(i),0.02,'r-');
end
grid on;
legend(h([1 2 3 4]),"Path planned","Simulation path",...
    "Origin","Goal",'Location','southeast')
hold off
%
R=195e-3/2;
L=381e-3;
vR = (2*v0+out(:,3)*L)/(2*R);
vL = (2*v0-out(:,3)*L)/(2*R);
wSat=2.4;
vRSat=(2*v0+wSat*L)/(2*R);
vLSat=(2*v0-wSat*L)/(2*R);
figure();
h(1) = plot(timeSim,vR,'LineWidth',2); grid, hold on;
h(2) = plot(timeSim,vL,'LineWidth',2);
h(3) = yline(vRSat,'--k','LineWidth',2);
yline(vLSat,'--k','LineWidth',2);
xlabel("t [s]");
ylabel("Angular velocity [rad/s]");
title({'{\bf Angular velocities of the DDMR wheels with P control law}'; ...
        ['{\it\fontsize{10} K_p} = ',num2str(Kp)]},...
         'FontWeight','Normal');
legend(h([1 2 3]),"Right wheel","Left wheel",...
    "Saturation",'Location','southeast')
hold off
%% Functions

function zDot = dinModel(t,z)
    global Kp v0 goal dMin
        
    xDot = v0*cos(z(3));
    yDot = v0*sin(z(3));
    phiRef = atan2((goal(2)-z(2)),(goal(1)-z(1)));
    e = unwrap(phiRef - z(3));
    phiDot = Kp*e;
    
    d = sqrt((goal(2)-z(2))^2+(goal(1)-z(1))^2);
    
    if(d < dMin)
        v0 = 0;
    end
     
    zDot = [xDot;yDot;phiDot];
end