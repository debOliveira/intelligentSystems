%% Clearing workspace
clear all
close all
clc
%% global variables
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
        fprintf('Starting maze.m\n');
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
while(e~=0)
	[e,sensorU1]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
	[e,sensorU4]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
    [e,sensorU5]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
	[e,sensorU8]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_blocking);
end
e=-1;
while (e~=0)
    [e,goalHandle]=sim.simxGetObjectHandle(clientID,'Goal',sim.simx_opmode_blocking);
end
e=-1;
while (e~=0)
    [e,leftMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
end
e=-1;
while (e~=0)
	[e,rightMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
end
fprintf('Got handles...\n');
%% Defining coppeliaSim client side parameters
np=5000;
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
DR=zeros(np,1);
DL=zeros(np,1);
DF=zeros(np,1);
vel=zeros(np,1);
omega=zeros(np,1);
latest=zeros(2,1);
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
e = 1;
while(e~=0)
    [e,goal]= sim.simxGetObjectPosition(clientID,goalHandle,-1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    	[e,~,~,~,~]=sim.simxReadProximitySensor(clientID,sensorU1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    	[e,~,~,~,~]=sim.simxReadProximitySensor(clientID,sensorU4,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    	[e,~,~,~,~]=sim.simxReadProximitySensor(clientID,sensorU5,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    	[e,~,~,~,~]=sim.simxReadProximitySensor(clientID,sensorU8,sim.simx_opmode_streaming);
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
    [~,e1,d1,~,~]=sim.simxReadProximitySensor(clientID,sensorU1,sim.simx_opmode_buffer);
    [~,e4,d4,~,~]=sim.simxReadProximitySensor(clientID,sensorU4,sim.simx_opmode_buffer);
    [~,e5,d5,~,~]=sim.simxReadProximitySensor(clientID,sensorU5,sim.simx_opmode_buffer);
    [~,e8,d8,~,~]=sim.simxReadProximitySensor(clientID,sensorU8,sim.simx_opmode_buffer);
    %% Saving robot pose - rob
    if(~e1)
        d1(3) = 2.5;
    end
    if(~e4)
        d4(3) = 2.5;
    end
    if(~e5)
        d5(3) = 2.5;
    end
    if(~e8)
        d8(3) = 2.5;
    end

    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id)=robOri(1,3);
    DR(id)=d8(3);
    DL(id)=d1(3);
    DF(id)=min(d4(3),d5(3));
    %% Controlling
    if DR(id) <= DL(id)
        DS = -DR(id);
    else
        DS = DL(id);
    end
    DSlim = DS;
    DSlim(DS > 2) = 2;
    DSlim(DS < -2) = -2;

    DFlim=DF(id);
    DFlim(DF(id) > 2) = 2;

    ObstacleDistance = DFlim;
    SideObstacleDistance = DSlim;        
    GoalDistance = sqrt((goal(1)-robPos(1))^2+(goal(2)-robPos(2))^2); 
    if (GoalDistance>10)
        GoalDistance = 10;
    end
    SteeringAngle = atan2(goal(2)-robPos(2),goal(1)-robPos(1))-(robOri(3));
    SteeringAngle = wrapTo180(rad2deg(SteeringAngle));
        
    if GoalDistance < 0.02
        result = [0 0];
        disp("On Goal");
        break;
    elseif ( (xor(DL(id) < 2,DR(id) < 2) && DF(id) < 2) || (DL(id) > 2 && DR(id) > 2 && DF(id) < 2))
        result = evalfis(ObstacleAvoidance,[ObstacleDistance SideObstacleDistance]);
        disp("ObstacleAvoidance");
    elseif ( (DL(id) < 2 && DR(id) < 2 && DF(id) >= 2) || ((DL(id) < 2 || DR(id) < 2) && DF(id) >= 2))
        result = evalfis(Tracking,[SteeringAngle SideObstacleDistance]);
        disp("Tracking");
    elseif (DL(id) < 2 && DR(id) < 2 && DF(id) < 2)
        result = evalfis(DeadlockDisarming,[ObstacleDistance SteeringAngle]);
        disp("DeadlockDisarming");
    else 
        result = evalfis(GoalSeeking,[GoalDistance SteeringAngle]);
        disp("GoalSeeking");
    end
        
    w = deg2rad(result(1));
    v = result(2);
    vel(id) = v;
    omega(id) = w;
    
    leftVel = (2*v-w*L)/2/R;
    rightVel = (2*v+w*L)/2/R;
    %% Actuating
    [~] = sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel, sim.simx_opmode_oneshot);
    [~] = sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel, sim.simx_opmode_oneshot);
    %% Saving
    %dotPhiR(id)= phiRCop(1,1);
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
%% Plotting
% plot(DR)
% hold on
% plot(DL)
% hold on
% plot(DF)
% legend("DR","DL","DF")
lineStyles = linspecer(4);

figure('Position', [10 10 500 450]);
h(1) = scatter(-xp(1)+5,-yp(2)+5,300,"x",...
               'MarkerEdgeColor',lineStyles(2,:),...
               'LineWidth',2.0); 
grid, hold on;
h(2) = scatter(-goal(1)+5,-goal(2)+5,500,"o",...
               'MarkerEdgeColor',lineStyles(3,:),...
               'LineWidth',2.0); 
h(3) = plot(-xp(1:id)+5*ones(id,1),-yp(1:id)+5*ones(id,1),'--',...
            'color',lineStyles(1,:),'LineWidth',2.0);
for i=1:round(id/40):id
    drawRobot(-xp(i)+5,-yp(i)+5,fp(i)+pi,0.02,lineStyles(1,:));
end
load("maze_sensors2and7.mat");
h(4) = plot(-xp(1:2299)+5*ones(2299,1),-yp(1:2299)+5*ones(2299,1),'--',...
            'color',lineStyles(4,:),'LineWidth',2.0);
for i=1:round(2299/40):2299
    drawRobot(-xp(i)+5,-yp(i)+5,fp(i)+pi,0.02,lineStyles(4,:));
end
grid on;
axis([0 12 -2 10]);
rectangle('Position',[0.9 1 0.2 8],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[2.9 1 0.2 6],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[4.9 0 0.2 7],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[6.9 3 0.2 6],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[8.9 0 0.2 9],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[3 6.8 2 0.2],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[1 8.8 6 0.2],...
          'FaceColor','k','EdgeColor','k');
rectangle('Position',[5 0 4 0.2],...
          'FaceColor','k','EdgeColor','k');
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR}'},...
         'FontWeight','Normal');
legend(h([3 4 1 2]),"Path (Sensors 1 and 8)",...
    "Path (Sensors 2 and 7)","Origin","Goal",'Location','southeast')
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
saveas(gca,"pics/"+"maze"+".png")