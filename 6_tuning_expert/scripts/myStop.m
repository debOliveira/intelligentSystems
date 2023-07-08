function myStop()
    gains  = evalin('base', 'gains');
    %
    nh=[0 -1 2];
    dh=conv([10 1],conv([10 1],[1 2]));
    %
    h=1e-3;
    t=[0:h:100]';
    %    
    figure('Position',[10 10 500 300]),hold on
    %
    lineStyles = linspecer(2,'sequential');
    %
    Kc = gains(1,1);
    Ti = gains(1,2);
    Td = gains(1,3);
    %
    ng=[Kc*Ti*Td Kc*Ti Kc];
    dg=[0 Ti 0];
    %
    [nl,dl]=series(ng,dg,nh,dh);
    [num,den]=cloop(nl,dl,-1);
    %
    [y,~]=step(num,den,t);
    h(1)=plot(t,y,'color',lineStyles(1,:),...
        'LineWidth',1.5);
    for i=2:2:9
        Kc = gains(i,1);
        Ti = gains(i,2);
        Td = gains(i,3);
        %
        ng=[Kc*Ti*Td Kc*Ti Kc];
        dg=[0 Ti 0];
        %
        [nl,dl]=series(ng,dg,nh,dh);
        [num,den]=cloop(nl,dl,-1);
        %
        [y,~]=step(num,den,t);
        plot(t,y,'k');
    end
    for i=1:(length(gains)/5-1)
        Kc = gains(i*5,1);
        Ti = gains(i*5,2);
        Td = gains(i*5,3);
        %
        ng=[Kc*Ti*Td Kc*Ti Kc];
        dg=[0 Ti 0];
        %
        [nl,dl]=series(ng,dg,nh,dh);
        [num,den]=cloop(nl,dl,-1);
        %
        [y,~]=step(num,den,t);
        plot(t,y,'k');
    end
    Kc = gains(length(gains),1);
    Ti = gains(length(gains),2);
    Td = gains(length(gains),3);
    %
    ng=[Kc*Ti*Td Kc*Ti Kc];
    dg=[0 Ti 0];
    %
    [nl,dl]=series(ng,dg,nh,dh);
    [num,den]=cloop(nl,dl,-1);
    %
    [y,~]=step(num,den,t);
    h(2)=plot(t,y,'color',lineStyles(2,:),...
        'LineWidth',1.5);
    legend(h([1 2]),"First","Last")
    grid, hold off;   
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset; 
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
end