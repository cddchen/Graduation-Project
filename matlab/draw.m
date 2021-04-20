function [] = draw(x1, y1, z1, x2, y2, z2)
    %x1:0,x2:2;y1:1,y2:2;z1:0;z2:1
    verts=[x1 y1 z1;
       x2 y1 z1;
       x2 y2 z1;
       x1 y2 z1;
       x1 y1 z2;
       x2 y1 z2;
       x2 y2 z2;
       x1 y2 z2];

    % 6个面顶点编号
    faces=[1 2 3 4;
           1 2 6 5;
           2 3 7 6;
           3 4 8 7;
           4 1 5 8;
           5 6 7 8];
    % 作图
    patch('Faces',faces,'Vertices',verts,'Facecolor','none',...
        'LineWidth',1.5,'EdgeColor','blue');
    hold on;
end
