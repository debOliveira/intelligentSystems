close all; clc; clear all;
%% Time span
tStart = 0;
tEnd = 5;
timeSpan = [tStart tEnd];
%% Defining variables
global Kb Ra La R L mc d N Kt M B

%Actuator
Ra  = 0.71;      %Ohms
La  = 0.66e-3;   %H
Kb  = 0.023;     %V/(rad-s)
Kt  = 0.029;     %N-m/A
N   = 38.3;

%Body
R   = 0.1;      %m
L   = 0.38/2;   %m
mw  = 1;        %Kg
mc  = 7;        %Kg
m   = mc + 2*mw;%Kg
d   = 0.05;     %m
Im  = 0.0025;   %Kg-m^2
Ic  = 1;        %Kg-m^2
Iw  = 0.005;    %Kg-m^2
I   = Ic + mc*d^2 + 2*mw*L^2+2*Im; %Kg-m^2

%Dynamic eq.
M =[Iw + R^2/(4*L^2)*(m*L^2+I), ...
    R^2/(4*L^2)*(m*L^2-I);...
    R^2/(4*L^2)*(m*L^2-I),...
    Iw + R^2/(4*L^2)*(m*L^2+I)];
B   = [1 0;0 1];    
%% Initial conditions
initPhiR_Dot = 0;
initPhiL_Dot = 0;
n = [initPhiR_Dot; initPhiL_Dot];

initTheta = 0;
initPos = [2;2];

initIaR = 0;
initIaL = 0;
i = [initIaR;initIaL];

x=[n;i;initTheta;initPos];
%% Solving ODE
[t,out] = ode45(@dinModel,timeSpan,x);
%% Plotting Data
% subplot(2,1,1)
% plot(t,out(:,3)), grid;
% xlabel('$t\;[s]$',"Interpreter","latex");
% ylabel('$I_{a_R}\;[A]$',"Interpreter","latex");
% title("Armature current from right motor");
% 
% subplot(2,1,2);
% plot(t,out(:,4)), grid;
% xlabel('$t\;[s]$',"Interpreter","latex");
% ylabel('$I_{a_L}\;[A]$',"Interpreter","latex");
% title("Armature current from left motor");

figure();
subplot(2,1,1);
plot(t,out(:,6),t,out(:,7),t,out(:,5),'LineWidth',2),grid
legend('$x(t)$','$y(t)$','$\phi(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Pose of the DDMR in the dynamic model");

subplot(2,1,2);
plot(t,out(:,1),t,out(:,2),'LineWidth',2),grid
legend('$\phi_R(t)$','$\phi_L(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Angular velocities of the DDMR wheels in the dynamic model");
%% Exporting to Coppelia
timeSim = t;
timeSeries = timeseries([out(:,1)';out(:,2)';out(:,6)';out(:,7)';out(:,5)']',t);
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
[rLJoint, leftMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
[rRJoint, rightMotor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
%% Defining coppeliaSim client side parameters
np=100;
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
torqueRp=zeros(np,1);
torqueLp=zeros(np,1);
phiDotRp=zeros(np,1);
phiDotLp=zeros(np,1);
timeResampled = resample(timeSeries,0:hd:tf);
%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);
%% Get initial pose = [position orientation]
e=1;
while (e~=0)
    [e,robPosI]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_streaming);
end
e=1;
while (e~=0)
    [e,robOriI]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_streaming);
end
%
%% Main control loop - coppeliaSim client side
while (td<tf)
    %% Current sampling instant
    t(id)=td;
    %% Castle wheel debugging
%     if and(td<0.7,td>=0.4)
%         keyboard;
%     end
    %% Measuring
    [~,robPos]=sim.simxGetObjectPosition(clientID,rob,-1,sim.simx_opmode_buffer);
    [~,robOri]=sim.simxGetObjectOrientation(clientID,rob,-1,sim.simx_opmode_buffer);
    if (td>0)
        [~,robPhiDotR]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_buffer);
        [~,robPhiDotL]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_buffer);
        [~,robTorqueR]=sim.simxGetJointForce(clientID,rightMotor,sim.simx_opmode_buffer);
        [~,robTorqueL]=sim.simxGetJointForce(clientID,leftMotor,sim.simx_opmode_buffer);
    else
        [~,robPhiDotR]=sim.simxGetObjectFloatParameter(clientID,rightMotor,2012,sim.simx_opmode_streaming);
        [~,robPhiDotL]=sim.simxGetObjectFloatParameter(clientID,leftMotor,2012,sim.simx_opmode_streaming);
        [~,robTorqueR]=sim.simxGetJointForce(clientID,rightMotor,sim.simx_opmode_streaming);
        [~,robTorqueL]=sim.simxGetJointForce(clientID,leftMotor,sim.simx_opmode_streaming);
    end
    %% Controlling
    leftVel= timeResampled.Data(id,2);
    rightVel= timeResampled.Data(id,1);
    %% Actuating
    [~] = sim.simxSetJointTargetVelocity(clientID, leftMotor, leftVel, sim.simx_opmode_oneshot);
    [~] = sim.simxSetJointTargetVelocity(clientID, rightMotor, rightVel, sim.simx_opmode_oneshot);
    %% Saving
    %dotPhiR(id)= phiRCop(1,1);
    xp(id)=robPos(1,1);
    yp(id)=robPos(1,2);
    fp(id)=robOri(1,3);
    torqueRp(id)=robTorqueR;
    torqueLp(id)=robTorqueL;  
    phiDotRp(id)=robPhiDotR(1);
    phiDotLp(id)=robPhiDotL(1);    
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
figure();
subplot(2,1,1);
plot(t,xp,t,yp,t,fp,'LineWidth',2),grid
legend('$x(t)$','$y(t)$','$\theta(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Pose of the DDMR in simulation");

subplot(2,1,2);
plot(t,phiDotRp,t,phiDotLp,'LineWidth',2),grid
legend('$\phi_R(t)$','$\phi_L(t)$',"Interpreter","latex",...
        "Location","southeast");
xlabel('t [s]',"Interpreter","latex")
title("Angular velocities of the DDMR wheels in simulation");

%% Comparing both models
figure();
axis equal
hold on %hold for drawRobot(.)
h(1) = plot(timeResampled.Data(:,3),timeResampled.Data(:,4),'b:','LineWidth',2.0);
xlabel('$x\;[m]$',"Interpreter","latex");
ylabel('$y\;[m]$',"Interpreter","latex");
for i=1:round(size(timeResampled.Data,1)/20):size(timeResampled.Data,1)
    drawRobot(timeResampled.Data(i,3),timeResampled.Data(i,4),timeResampled.Data(i,5),0.02,'b-');
end
h(2) = plot(xp,yp,'r:','LineWidth',2.0);
for i=1:round(length(xp)/20):length(xp)
    drawRobot(xp(i),yp(i),fp(i),0.02,'r-');
end
legend(h([1 2]),"Dynamic Model","Simulation");
grid on;
hold off %release it

% figure();
% subplot(2,1,1);
% plot(t,torqueRp,timeSim,N*Kt*out(:,3),'LineWidth',2),grid
% legend("Dynamic Model","Simulation","Location","southeast");
% xlabel('t [s]',"Interpreter","latex");
% ylabel('$\tau_R\;[N\cdot m]$',"Interpreter","latex")
% title("Torque of the right wheel");
% 
% subplot(2,1,2);
% plot(t,torqueLp,timeSim,N*Kt*out(:,4),'LineWidth',2),grid
% legend("Dynamic Model","Simulation","Location","southeast");
% xlabel('t [s]',"Interpreter","latex")
% ylabel('$\tau_L\;[N\cdot m]$',"Interpreter","latex")
% title("Torque of the left wheel");
%% Functions
function v = Va(t)
    if t < 1
        v = [-3;-3];
    else
        v = [-3;-3];
    end
end

function xDot = dinModel(t,x)
    global R L mc d Kb Ra La N Kt M B
        
    tau = N*Kt*[x(3);x(4)];
    
    thetaDot = R/(2*L)*(x(1) - x(2)); 
    V = [0 R^2/(2*L)*mc*d*thetaDot; ...
         -R^2/(2*L)*mc*d*thetaDot 0];
    
    n = [x(1);x(2)];
    nDot = M\(B*tau - V*n);    
    
    ea = Kb*N*n;
    ia = [x(3);x(4)];     
    iaDot = (Va(t)-ea-Ra*ia)/La;
    
    xPosDot = R/2*(x(1) + x(2))*cos(x(5));
    yPosDot = R/2*(x(1) + x(2))*sin(x(5));   
    
    xDot = [nDot;iaDot;thetaDot;xPosDot;yPosDot];
end