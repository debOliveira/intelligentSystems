%% Clearing workspace
clear all
close all
clc
rng(pi)
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
        fprintf('Starting tracking.m\n');
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
	[e,sensorU1]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',sim.simx_opmode_blocking);
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
	[e,sensorU8]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
    [e,goalHandle]=sim.simxGetObjectHandle(clientID,'Goal',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
    [e,leftMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
end
e=1;
while (e~=0)
	[e,rightMotor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
end
fprintf('Got handles...\n');
%% Defining coppeliaSim client side parameters
np=1000;
hd=50e-3;
tf=np*hd;
tc=0;
td=0;
id=1;
xVar = 1;    
yVar = 1;    
thetaVar = 1; 
d1Var = 1;   
d2Var = 1;   
vVar = 1;    
wVar = 1;   
%
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
DR=zeros(np,1);
DL=zeros(np,1);
DF=zeros(np,1);
velo=zeros(np,1);
omegao=zeros(np,1);
latest=zeros(2,1);
d1p=zeros(np,1);
d2p=zeros(np,1);
v=zeros(np,1);
w=zeros(np,1);
Y = zeros(np,6);
t = zeros(np,1);
%% Initialization
Ds=0.0478;
D=195e-3;
Ls=0.0640;
Rd=D/2;
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
e = 1;
while(e~=0)
[e,~]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_streaming);
end
e = 1;
while(e~=0)
[e,~]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_streaming);
end
%
fprintf('First values read\n');
%% Initial Kalman Variables
T = 0.05;
xe = [robPos(1,1);robPos(1,2);robOri(1,3)];
P = 1e3*eye(3);
Q = diag([sqrt(vVar),sqrt(wVar)]);
R = diag([sqrt(xVar),sqrt(yVar),sqrt(thetaVar),sqrt(d1Var),sqrt(d2Var)]);
% Normal plane to origin
Pv1 = -1;
Pv2 = 1;
Pr1 = 1;
Pr2 = 1;
Pn1 = pi-pi/4;
Pn2 = Pn1;
% Sensor position to body frame
x1 = -0.12505;
y1 = 0.15543;
x2 = 0.12024;
y2 = 0.15555;
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
    DR(id)=d8(3);
    DL(id)=d1(3);
    DF(id)=min(d4(3),d5(3));  
    du1 = d1;
    du2 = d8;
    [~,wL]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_buffer );
    [~,wR]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_buffer );
    %% Saving robot pose - rob
    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id)=robOri(1,3);
    d1p(id)=du1(1,3);
    d2p(id)=du2(1,3);  
    v(id)=0.1/2*(wL+wR);
    w(id)=0.1/2*(-wL+wR);
    if(id>1)
        i = id;
        %% Adding Noise
        x_k = xp(id) + sqrt(xVar)*randn();
        y_k = yp(id) + sqrt(yVar)*randn();
        theta_kk = fp(id) + sqrt(thetaVar)*randn();
        d1_k = d1p(id) + sqrt(d1Var)*randn();
        d2_k = d2p(id) + sqrt(d2Var)*randn();%% Creating matrix
        X = [x_k'; y_k'; theta_kk']';  %states
        U = [v'; w']';          %inputs
        M = [d1_k'; d2_k']';        %measurements
        %% Kalman loop model initialization
        theta_k = xe(3);
        v_k = U(i-1,1);
        Ad_k = [1 0 -v_k*T*sin(theta_k);
                0 1 v_k*T*cos(theta_k);
                0 0 1];
        Lk = [T*cos(theta_k) -0.5*v_k*T^2*sin(theta_k);
             T*sin(theta_k) 0.5*v_k*T^2*cos(theta_k);
             0 T];
        Q_k = [T+v_k^2*T^3/3*sin(theta_k)^2  -v_k^2*T^3/3*sin(theta_k)*cos(theta_k) -v_k*T^2/2*sin(theta_k)
               -v_k^2*T^3/3*sin(theta_k)*cos(theta_k) T+v_k^2*T^3/3*cos(theta_k)^2 v_k*T^2/2*cos(theta_k)
               -v_k*T^2/2*sin(theta_k) v_k*T^2/2*cos(theta_k) T];
        Qd_k = 0.025*Q_k;
        C = [1 0 0;
            0 1 0; 
            0 0 1;
            -Pv1*(cos(Pn1)), -Pv1*(sin(Pn1)) , Pv1*(x1*sin(theta_k-Pn1)-y1*cos(theta_k-Pn1));
            -Pv2*(cos(Pn2)), -Pv2*(sin(Pn2)) , Pv2*(x2*sin(theta_k-Pn2)-y2*cos(theta_k-Pn2))];
        %Saving
        t(i-1,1)=i;
        Y(i-1,1)= xp(i);
        Y(i-1,2)= yp(i);
        Y(i-1,3)= double(x_k);
        Y(i-1,4)= double(y_k);
        aux = (C*xe)';    
        Y(i-1,5)= double(aux(1));
        Y(i-1,6)= double(aux(2));
        %Measurement
        ym=[X,M]';
        %Kalman Filter
        ye=C*xe;
        K=P*C'*inv(C*P*C'+R);
        xe=xe+K*(ym-ye);
        P=(eye(3)-K*C)*P;
        xe=xe+Lk*U(i,:)';
        P=Ad_k*P*Ad_k'+Qd_k;
        %% Controlling
        DSEqual = 0;
        if DR(id) < DL(id)
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
        GoalDistance = sqrt((goal(1)-x_k)^2+(goal(2)-y_k)^2);      
        SteeringAngle = atan2(goal(2)-y_k,goal(1)-x_k)-(theta_kk);        
%         GoalDistance = sqrt((goal(1)-xe(1))^2+(goal(2)-xe(2))^2);      
%         SteeringAngle = atan2(goal(2)-xe(2),goal(1)-xe(1))-(xe(3));
        SteeringAngle = wrapTo180(rad2deg(SteeringAngle));

        if GoalDistance < 0.1
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

        wo = deg2rad(result(1));
        vo = result(2);
        velo(id) = vo;
        omegao(id) = wo;

        leftVel = (2*vo-wo*L)/2/Rd;
        rightVel = (2*vo+wo*L)/2/Rd;
        %% Actuating
        [~] = sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel, sim.simx_opmode_oneshot);
        [~] = sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel, sim.simx_opmode_oneshot); 
    end
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
id=id-1;

figure('Position', [10 10 500 450]);
h(1) = scatter(xp(1),yp(2),300,"x",...
               'MarkerEdgeColor',lineStyles(2,:),...
               'LineWidth',2.0); 
grid, hold on;
h(2) = scatter(goal(1),goal(2),500,"o",...
               'MarkerEdgeColor',lineStyles(3,:),...
               'LineWidth',2.0); 
h(3) = plot(xp(1:id),yp(1:id),'--',...
            'color',lineStyles(1,:),'LineWidth',2.0);
for i=1:round(id/20):id
    drawRobot(xp(i),yp(i),fp(i),0.02,lineStyles(1,:));
end
load("dataFuzzy.mat");
h(4) = plot(xp(1:id),yp(1:id),'-.',...
            'color',lineStyles(4,:),'LineWidth',1.5);
for i=1:round(id/20):id
    drawRobot(xp(i),yp(i),fp(i),0.02,lineStyles(4,:));
end
axis([-3.5 4.5 -3.5 4.5]);
plot([-2.5 1],[-1 2.5], 'k', 'LineWidth',3.0);
plot([-1 2.5],[-2.5 1], 'k', 'LineWidth',3.0);
grid on;
xlabel("x [m]");
ylabel("y [m]");
title({'{\bf Path followed by the DDMR}'},...
         'FontWeight','Normal');
legend(h([3 4 1 2]),"No EFK",...
    "With EFK","Origin","Goal",'Location','southeast')
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
saveas(gca,"pics/"+"tracking"+".png")
%%
lineStyles = linspecer(3);
n = length(Y)-50;

figure('Position',[10 10 600 400])
subplot(2,1,1)
plot(t(1:n-1),Y(1:n-1,1),':','LineWidth',1.5,'color',lineStyles(1,:)), hold on
plot(t(1:n-1),Y(1:n-1,3),'LineWidth',1.5,'color',lineStyles(3,:))
plot(t(1:n-1),Y(1:n-1,5),'LineWidth',1.5,'color',lineStyles(2,:))
xlabel('# samples')
ylabel('meters')
legend("x(n)", "x_n(n)", "x_e(n)",...
    'location','best');
xlim([0 100]);
grid
subplot(2,1,2)
plot(t(1:n-1),Y(1:n-1,2),':','LineWidth',1.5,'color',lineStyles(1,:)), hold on
plot(t(1:n-1),Y(1:n-1,4),'LineWidth',1.5,'color',lineStyles(3,:))
plot(t(1:n-1),Y(1:n-1,6),'LineWidth',1.5,'color',lineStyles(2,:))
xlabel('# samples')
ylabel('meters')
legend("y(n)", "y_n(n)", "y_e(n)",...
    'location','best');
xlim([0 100]);
grid

[~,~,~] = mkdir('pics');
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
saveas(gca,"pics/"+name+".png")

lineStyles = linspecer(1);

figure('Position',[10 10 600 400])
subplot(2,1,1)
plot(t(1:n-1),Y(1:n-1,5)-Y(1:n-1,1),'LineWidth',1.5,'color',lineStyles(1,:))
ylabel('meters')
legend("x(n)-x_e(n)",...
    'location','best');
xlim([0 100]);
grid
subplot(2,1,2)
plot(t(1:n-1),Y(1:n-1,6)-Y(1:n-1,2),'LineWidth',1.5,'color',lineStyles(1,:))
xlabel('# samples')
legend("y(n)-y_e(n)",...
    'location','best');
ylabel('meters')
xlim([0 100]);
grid

saveas(gca,"pics/"+name+"_Error.png")