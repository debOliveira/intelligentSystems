%% Clearing workspace
clear all
close all
clc
randn('seed',0)
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
while(e~=0)
 [e,u1]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
 [e,u2]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
 [e,lJ]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
end
e=1;
while(e~=0)
 [e,rJ]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking);
end
%% Defining coppeliaSim client side parameters
np=500;
hd=50e-3;
tf=np*hd;
tc=0;
td=0;
id=1;
xVar = 0.0025;    
yVar = 0.0025;    
thetaVar = 0.0025; 
d1Var = 0.0025;   
d2Var = 0.0025;   
vVar = 0.0025;    
wVar = 0.0025;   
%
t=zeros(np,1);
xp=zeros(np,1);
yp=zeros(np,1);
fp=zeros(np,1);
d1p=zeros(np,1);
d2p=zeros(np,1);
v=zeros(np,1);
w=zeros(np,1);
Y = zeros(np,6);
t = zeros(np,1);
%% Starting coppeliaSim simulation
[res]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait);
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
[e,~,d,~,~]=sim.simxReadProximitySensor(clientID,u1,sim.simx_opmode_streaming);
end
e = 1;
while(e~=0)
[e,~,~,~,~]=sim.simxReadProximitySensor(clientID,u2,sim.simx_opmode_streaming);
end
e = 1;
while(e~=0)
[e,~]=sim.simxGetObjectFloatParameter(clientID,lJ,2012,sim.simx_opmode_streaming);
end
e = 1;
while(e~=0)
[e,~]=sim.simxGetObjectFloatParameter(clientID,rJ,2012,sim.simx_opmode_streaming);
end
%
%% Initial Kalman Variables
T = 0.05;
xe = [robPos(1,1);robPos(1,2);robOri(1,3)];
P = 1e3*eye(3);
Q = diag([sqrt(vVar),sqrt(wVar)]);
R = diag([sqrt(xVar),sqrt(yVar),sqrt(thetaVar),sqrt(d1Var),sqrt(d2Var)]);
% Normal plane to origin
Pv1 = -1;
Pv2 = 1;
Pr1 = sqrt(4.5^2+4^4);
Pr2 = 0.92491;
Pn1 = pi;
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
    [~,~,du1,~,~]=sim.simxReadProximitySensor(clientID,u1,sim.simx_opmode_buffer );
    [~,~,du2,~,~]=sim.simxReadProximitySensor(clientID,u2,sim.simx_opmode_buffer );
    [~,wL]=sim.simxGetObjectFloatParameter(clientID,lJ,2012,sim.simx_opmode_buffer );
    [~,wR]=sim.simxGetObjectFloatParameter(clientID,rJ,2012,sim.simx_opmode_buffer );
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
        x = xp(id) + sqrt(xVar)*randn();
        y = yp(id) + sqrt(yVar)*randn();
        theta = fp(id) + sqrt(thetaVar)*randn();
        d1 = d1p(id) + sqrt(d1Var)*randn();
        d2 = d2p(id) + sqrt(d2Var)*randn();
        %% Creating matrix
        X = [x'; y'; theta']';  %states
        U = [v'; w']';          %inputs
        M = [d1'; d2']';        %measurements
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
        Y(i-1,3)= double(x);
        Y(i-1,4)= double(y);
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
%% Plotting results
% figure(2)
% axis equal
% hold on
% plot(xp,yp,'k:','LineWidth',2.0),grid
% xlabel('x_{p}(n), m'),ylabel('y_{p}(n), m')
% for i=1:round(length(xp)/20):length(xp)
%     drawRobot(xp(i),yp(i),fp(i),0.02);
% end
% hold off

lineStyles = linspecer(3);
n = length(Y);

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