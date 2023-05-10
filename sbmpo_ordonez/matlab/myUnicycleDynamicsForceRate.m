function dq = myUnicycleDynamicsForceRate(t,q,u,params);
m = params.m;
b = params.b;
fload = params.fload;

X = q(1);
Y = q(2);
th = q(3);
dX = q(4);
dY = q(5);
vx = q(6);

Fx = u(1);
dth = u(2);


dvx = (Fx-fload-b*vx)/m;

vy = 0; % no side slip
dvy = 0;

ax = dvx - dth*vy;
ay = dvy + dth*vx;

d2X = ax*cos(th) - ay*sin(th);
d2Y = ax*sin(th) + ay*cos(th);


dq = [dX;dY;dth;d2X;d2Y;dvx];

