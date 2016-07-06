function y = Ear2Geo(u)
%
global glv;

if length(u) > 3
	x = u(2); y = u(3); z = u(4);
else 
	x = u(1); y = u(2); z = u(3);
end

%
K_MAX = 10;
%

% if x > 0
%     if y > 0
%         long = atan(y/x);
%     else
%          long = 2*pi + atan(y/x);
%     end
% else
%     long = pi + atan(y/x);  
% end

long = atan2(y,x);
%
m=sqrt(x^2+y^2);
 
A = z*glv.Rp; B = 2*glv.Re*m; C = 2*(glv.Re^2-glv.Rp^2); 
%
num=0;
t = glv.Re*z/glv.Rp/m;
if(t ~=0)
    t = (sqrt(1+t^2)-1)/t;
    tmp = 1e8;
    % 
    while abs(t-tmp)>1e-9
        tmp = t;
        t = t-(A*t^4+(B+C)*t^3+(B-C)*t-A)/(4*A*t^3+3*(B+C)*t^2+B-C);
        num = num+1;     
        if(num>K_MAX)
            num =  20;
            break;          
        end               
    end
end

%
if t==0
    lat = 0;
    h = m-glv.Re;
else       
    k=1/((glv.Re/glv.Rp)*(2*t/(1-t^2)));
    lat = atan(1/k);
    h = sign(lat)*(z-glv.Rp*(2*t/(1+t^2)))*sqrt(1+k^2);
end
%     
y= [long, lat, h];
          