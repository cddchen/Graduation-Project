% read the obstacles
box_size = 0;
cnt = 0;
box = [];
st = [];
ed = [];
path_size = 0;
x = [];
y = [];
z = [];
fidin = fopen('../out4.txt');
while ~feof(fidin)
    tline = fgetl(fidin);
    if cnt == 0
        st = sscanf(tline, "(%d, %d, %d)");
    elseif cnt == 1
        ed = sscanf(tline, "(%d, %d, %d)");
    elseif cnt == 2
        box_size = sscanf(tline, "%d");
    elseif cnt <= 2 + box_size
        abox = sscanf(tline, "[(%d, %d), (%d, %d), (%d, %d), (%d, %d)], (%d, %d)");
        %box = [box; [abox(1) abox(2) abox(3) abox(4) abox(5) abox(6)]];
        draw3D(abox(1),abox(2),abox(3),abox(4),abox(5),abox(6),abox(7),abox(8),abox(9),abox(10));
    elseif cnt == 3 + box_size
        path_size = sscanf(tline, "%d");
    elseif cnt <= 3 + box_size + path_size
        point = sscanf(tline, "(%d, %d, %d)");
        x = [x point(1)];
        y = [y point(2)];
        z = [z point(3)];
    end
    cnt = cnt + 1;
end
fclose(fidin);
% draw the box
%{
max_x = max(st(1), ed(1));
max_y = max(st(2), ed(2));
max_z = max(st(3), ed(3));
for i = 1:box_size
    draw(box(i,1),box(i,2),box(i,3),box(i,4),box(i,5),box(i,6));
    max_x = max(max_x, box(i, 4));
    max_y = max(max_y, box(i, 5));
    max_z = max(max_z, box(i, 6));
end
%}
% draw the segment
plot3(x, y, z, 'Color', 'red', 'lineWidth', 1);
plot3(st(1), st(2), st(3), '-o', 'Color', 'black', 'MarkerSize', 5, 'MarkerFaceColor','#DC143C');
plot3(ed(1), ed(2), ed(3), '-o', 'Color', 'black', 'MarkerSize', 5, 'MarkerFaceColor','#DC143C');
% draw the axis
xlabel('x')
ylabel('y')
zlabel('z')
% axis([-0.5 max_x + 1 -0.5 max_y + 1 -0.5 max_z + 1])