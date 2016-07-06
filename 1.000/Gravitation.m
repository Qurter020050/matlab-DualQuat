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
	h = pos(3);
	
g = 9.7803267711905*(1+0.00193185138639*sin(lat)^2)/(1-0.00669437999013*sin(lat)^2)^0.5;
gh = g/(1+h/glv.Re)^2;
gL = [0 0 0 -gh];
gE = FrameTrans(q,gL,1);
y = gE - glv.wie^2*[0 u(1) u(2) 0];
