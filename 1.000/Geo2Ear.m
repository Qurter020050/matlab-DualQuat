function y = Geo2Ear(u)		%% 本函数用于将地理系转化为地球系，以右前上的顺序描述

global glv;


long = u(1); lat = u(2); h = u(3);

Rn = glv.Re/sqrt((1-glv.f)^2*sin(lat)^2+cos(lat)^2);

y(1) = (Rn+h)*cos(lat)*cos(long);
y(2) = (Rn+h)*cos(lat)*sin(long);
y(3) = (Rn*(1-glv.f)^2+h)*sin(lat);
