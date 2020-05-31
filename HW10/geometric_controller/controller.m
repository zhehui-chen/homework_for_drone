function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);

xd_1dot = [cos(t); -sin(t); 0];
xd_2dot = [-sin(t); -cos(t); 0];

%calculate errors 
ex = x-xd;
ev = v-xd_1dot;

%inertial frame
e3 = [0; 0; 1];

%thrust magnitude control
A = -P.kx*ex - P.kv*ev - P.gravity*e3 + P.mass*xd_2dot;
f = dot(-A, R*e3);

%desired R and omega
b3c = -(A)/norm(A);
b1c = -(cross(b3c, cross(b3c, b1d)))/norm(cross(b3c, b1d));
Rc_1dot = [0 0 0; 0 0 0; 0 0 0];

Rc = [b1c.'; cross(b3c, b1c).'; b3c.'];
Omegac = vee(Rc.'*Rc_1dot);

%inertia matrix
J = diag([P.Jxx P.Jyy P.Jzz]);

%error
eR = (1/2)*vee(Rc.'*R - R.'*Rc);
eOmega = Omega - R.'*Rc*Omegac;

%moment vector controll
M = -P.kR*eR - P.kOmega*eOmega + cross(Omega, J*Omega);

%calculate SO(3) error function, Psi
Psi = (1/2)*trace(eye(3) - Rc.'*R);

out = [f;M;eR;eOmega;Psi];
end