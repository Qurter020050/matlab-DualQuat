function y = Q_E2G_New(u)

global R;
global r;

long = u(1); lat = u(2); h = u(3);

% %参考《袁信》
% %地理坐标系采用北(Nx)、天(Cy)、东(Ez)
% %%单位天矢量
% fai = atan(r*tan(lat)/R);
% C = [cos(fai)*cos(long)/R cos(fai)*sin(long)/R sin(fai)/r];
% C = C/norm(C);
% %%单位东矢量
% z = [0 0 1];
% E = cross(z,C);
% E = E/norm(E);
% %% 单位北矢量
% N = cross(C,E);
% N = N/norm(N);
% %%地理系与地球系的关系
% Meg = [N;C;E];
% a = 0.5*sqrt(1+Meg(1,1)+Meg(2,2)+Meg(3,3));
% b = (Meg(2,3)-Meg(3,2))/4/a;
% c = (Meg(3,1)-Meg(1,3))/4/a;
% d = (Meg(1,2)-Meg(2,1))/4/a;

% %
% % 1. Rotating angle = longitude about Z axis
% % 2. Rotating angle = -(90+latitude) about Y axis
% % 3. Rotating angle = -90 about X axis

sum_lat = cos(lat/2)+sin(lat/2);
sub_lat = cos(lat/2)-sin(lat/2);

a = cos(long/2)*sub_lat+sin(long/2)*sum_lat;
b = -cos(long/2)*sub_lat+sin(long/2)*sum_lat;
c = -cos(long/2)*sum_lat-sin(long/2)*sub_lat;
d = -cos(long/2)*sum_lat+sin(long/2)*sub_lat;
y = 1/2*[a b c d];

