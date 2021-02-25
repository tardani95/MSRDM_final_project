function plotcfs(HT,frame_base_text,start_numbering_from)
    % % Visualize the cage and its respective coordinate frames
    tOff=5;
    aLength=25;

    grid on
    hold on

    axisX_0=[aLength;0;0];
    axisY_0=[0;aLength;0];
    axisZ_0=[0;0;aLength];

    % % Visualize the Origin O_0

    O_W=[0;0;0];

    plot3(O_W(1),O_W(2),O_W(3), 'k .','MarkerSize',30)
    text(O_W(1)+tOff,O_W(2)+tOff,O_W(3)+tOff, 'O_W');
    %Plot x-axis
    plot3([O_W(1);axisX_0(1)],[O_W(2);axisX_0(2)],[O_W(3);axisX_0(3)],'r -', 'Linewidth',2)
    %Plot y-axis
    plot3([O_W(1);axisY_0(1)],[O_W(2);axisY_0(2)],[O_W(3);axisY_0(3)],'g -', 'Linewidth',2)
    %Plot z-axis
    plot3([O_W(1);axisZ_0(1)],[O_W(2);axisZ_0(2)],[O_W(3);axisZ_0(3)],'b -', 'Linewidth',2)

    [~,~,n]=size(HT);

    for i=1:n
        % % Visualize the Origin Oi_0
        Oi_0(:,i)=HT(:,:,i)*[O_W;1];
        axisX1_0=HT(:,:,i)*[axisX_0;1];
        axisY1_0=HT(:,:,i)*[axisY_0;1];
        axisZ1_0=HT(:,:,i)*[axisZ_0;1];

        % Origin CF1
        plot3(Oi_0(1,i),Oi_0(2,i),Oi_0(3,i), 'k .','MarkerSize',30)
        text(Oi_0(1,i)+tOff,Oi_0(2,i)+tOff,Oi_0(3,i)+tOff, [frame_base_text num2str(start_numbering_from-1+i)]);
        %Plot x-axis
        plot3([Oi_0(1,i);axisX1_0(1)],[Oi_0(2,i);axisX1_0(2)],[Oi_0(3,i);axisX1_0(3)],'r -', 'Linewidth',2)
        %Plot y-axis
        plot3([Oi_0(1,i);axisY1_0(1)],[Oi_0(2,i);axisY1_0(2)],[Oi_0(3,i);axisY1_0(3)],'g -', 'Linewidth',2)
        %Plot z-axis
        plot3([Oi_0(1,i);axisZ1_0(1)],[Oi_0(2,i);axisZ1_0(2)],[Oi_0(3,i);axisZ1_0(3)],'b -', 'Linewidth',2)
    end

    xlabel('X [mm]');
    ylabel('Y [mm]');
    zlabel('Z [mm]');
end