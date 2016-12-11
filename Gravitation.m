function y = Gravitation(pos, u, q)

	if nargin == 1
		q = Q_E2G(pos);
		u = Geo2Ear(pos);
	elseif nargin == 2
		q = Q_E2G(pos);
	end

global glv;

	long = pos(1);
	lat = pos(2);
	h = glv.Hint;
	
%  g = 9.7803267711905*(1+0.00193185138639*sin(lat)^2)/(1-0.00669437999013*sin(lat)^2)^0.5;
g = glv.g0*(1+5.27094e-3*sin(pos(2))^2+2.32718e-5*sin(pos(2))^4)-3.086e-6*glv.Hint; % grs80
% gh = g/(1+h/glv.Re)^2;
gL = [0 0 0 -g];
gE = FrameTrans(q,gL,-1);
y = gE - glv.wie^2*[0 0 u(2) u(3)];
