% t=sin(30)cos(45)a1+sin(30)sin(45)a2+cos(30)a3
%Exercise 2--> give Rdes if you have t
t=[sin(pi/6)*cos(pi/4); sin(pi/6)*sin(pi/4); cos(pi/6)];
normt=sqrt(t(1)^2+t(2)^2+t(3)^2);
b3=[0;0;1];
psi=pi/4;
theta=pi/6;
phi=0;
Rdes=[cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta) -cos(phi)*sin(psi) cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
      cos(theta)*sin(psi)-cos(psi)*sin(phi)*sin(theta) cos(phi)*cos(psi) sin(psi)*sin(theta)+cos(theta)*sin(phi)*cos(psi);
      -cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)];
%Exercise 3--> give eR if you have Rdes and R
R=[0.7244 0.1294 0.6771;
   0.6424 -0.483 -0.595;
   0.25 0.866 -0.433];
Rdes=[0 0 1; 1 0 0; 0 1 0];
eR=R'*Rdes
  