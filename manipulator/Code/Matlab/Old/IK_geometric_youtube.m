syms Zc Xc

n = [0.11; 0.14; 133.37];
n = n/norm(n);

alpha = n(1);
beta = n(2);
gamma = n(3);

%find platform edge location
L = 70.6;
h = 133.37;

Ye = 0; %since the joints are revolute, the y value will never change
Ze = h + sqrt(L^2/(1+(gamma/alpha)^2));
Xe = sqrt(L^2 - (Ze-h)^2);

%ball joint location:
Yd = Ye;
Zd = Ze  - gamma*26.81;
Xd = Xe - alpha*26.81;

%======= Known: Xb, Yb, Zb (for the servo location. assume in y plane) ==========
Xb = sqrt(39.47^2 + 17.56^2);
Yb = 0;
Zb = 14.08;
%========================

% bearing location:
L1 = 51;
L2 = 72.58;
eq1 = (Xc-Xb)^2 + (Zc-Zd)^2 == L1^2;

eq2 = (Xc-Xb)^2 + (Zc-Zb)^2 == L2^2;

sol = vpasolve([eq1, eq2], [Xc,Zc]);

sol.Xc
sol.Zc

theta = atan2((sol.Zc-Zb), (sol.Xc-Xb))*180/3.1416





