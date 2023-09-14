
function PlotHoughLine(rho, theta, sStr)
    x1=rho*cos(theta);
    y1=rho*sin(theta);
    plot([0 x1],[0 y1],'r--');
    % x2==...
   % y2=....
   % x3=....
   % y3=..
   % plot([x2 x3],[y2 y3],'r-');
     x2= 10*cos(theta+pi/2);
     y2=10*sin(theta+pi/2);
   x3=x1-x2;
   y3=y1-y2;
   x2=x1+x2;
   y2=y1+y2;
   plot([x2 x3],[y2 y3],sStr,'LineWidth',2.0);
end