function [cf1,cf2,cf3,cf4,cf5,pico1,pico2] = myExpert(Kc,Ti,Td,p1,p2,period)
    % H(s)
    T=1;
    nh=[0 -1 2];
    dh=conv([10 1],conv([10 1],[1 2]));
    % G(s)
    % Kc=15;
    % Ti=6.75;
    % Td=1.69;
    %
    w1=30;
    w2=20;
    w3=1;
    w4=1;
    w5=20;
    %
    h=1e-3;
    t=0:h:100;
    t=t';
    %
    ng=[Kc*Ti*Td Kc*Ti Kc];
    dg=[0 Ti 0];
    %
    [nl,dl]=series(ng,dg,nh,dh);
    [num,den]=cloop(nl,dl,-1);
    %
    [y,~]=step(num,den,t);
    %
    [~,peaks] = findpeaks(y);
    [~,valeys] = findpeaks(-y);    
    if length(peaks) > 1
        pico1 = y(peaks(1));
        pico2 = y(peaks(2));
    else
       pico1=p1;
       pico2=p2;
    end
    if y(valeys(1)) < 0
        vale1 = y(valeys(2));
    else
        vale1 = y(valeys(1));
    end    
    %
    y100 = y - 1;
    zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0); 
    t100 = zci(y100);
    %       
    m = stepinfo(y,t);
    cf1 = m.Overshoot;%percentual de overshoot     
    cf2=abs((pico2-vale1)/(pico1-vale1));%amortecimento
    cf3 = m.RiseTime;%tempo de subida    
    cf4=2*(t(t100(2))-t(t100(1)));%período de oscilação
    cf5=abs((pico2-1)/(pico1-1));%razão de overshoot
%     ID=(w1*cf1+w2*cf2+w3*cf3+w4*cf4+w5*cf5);
    %
%     dy=[diff(y)./diff(t);0;0];
%     % figure(1),plot(t,y),grid
%     %
%     M=length(dy);
%     N=length(t);
%     DY=dy(1:M-1).* dy(2:M);
%     IP=find(DY <= 1e-4 & y > 1);
%     IV=find(DY <= 1e-4 & y > 0.1 & y < 1);
%     %
%     if length(IP) > 1
%        pico1=y(IP(1));
%        pico2=y(IP(2));
%     else
%        pico1=p1;
%        pico2=p2;
%     end
%     %
%     vale1=y(IV(1));
%     %
%     y010=0.1;
%     y010=y-y010;
%     DY010=y010(1:N-1).* y010(2:N);
%     t010=find(DY010 <= 0, 1 );
%     %
%     y090=0.9;
%     y090=y-y090;
%     DY090=y090(1:N-1).* y090(2:N);
%     t090=find(DY090 <= 0, 1 );
%     %
%     y100=1;
%     y100=y-y100;
%     DY100=y100(1:N-1).* y100(2:N);
%     t100=find(DY100 <= 0);
%     %
%     cf1=(pico1-1)*100;%percentual de overshoot 
%     cf2=(pico2-vale1)/(pico1-vale1);%amortecimento
%     cf3=t(t090)-t(t010);%tempo de subida
%     cf4=2*(t(t100(2))-t(t100(1)));%período de oscilação
%     cf5=(pico2-1)/(pico1-1);%razão de overshoot
%     ID=(w1*cf1+w2*cf2+w3*cf3+w4*cf4+w5*cf5)/(w1+w2+w3+w4+w5);
%     %
end
