function [] = draw(x0, y0, x1, y1, x2, y2, x3, y3)
    line([x0 x1 x2 x3 x0], [y0 y1 y2 y3 y0]);
    hold on;
end
