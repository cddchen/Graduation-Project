% 8个顶点xyz坐标
verts=[0 1 0;
       2 1 0;
       2 2 0;
       0 2 0;
       0 1 1;
       2 1 1;
       2 2 1;
       0 2 1];

% 6个面顶点编号
faces=[1 2 3 4;
       1 2 6 5;
       2 3 7 6;
       3 4 8 7;
       4 1 5 8;
       5 6 7 8];

xlabel('x')
ylabel('y')
zlabel('z')
% 作图
patch('Faces',faces,'Vertices',verts,'Facecolor','none',...
    'LineWidth',1.5,'EdgeColor','blue');
hold on;
plot3([1 2 3 3 3 3 2 1], [0 0 0 1 2 3 3 3], [0 0 0 0 0 0 0 0], 'Color', 'green', 'lineWidth', 1);
%plot3([1 3 3 1], [0 0 3 3], [0 0 0 0], 'Color', 'green', 'lineWidth', 1)
axis equal
axis([-0.5 5 -0.5 5 -0.5 5])
