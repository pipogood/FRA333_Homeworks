theta1 = 6.043218569544089 + pi; % +pi
theta2 = 3.6136940157364594;
theta3 = 0.21539221741744516;
a0 = 0;
a1 = 0;
a2 = 0.425;
d0 = 0.0892;
d1 = 0;
d2 = 0;
alpha0 = 0;
alpha1 = pi/2;
alpha2 = 0;

% homo0_1 = [cos(theta1) -sin(theta1)*cos(alpha0) sin(theta1)*sin(alpha0) a0*cos(theta1);
%     sin(theta1) cos(theta1)*cos(alpha0) -cos(theta1)*sin(alpha0) a0*sin(theta1);
%     0 sin(alpha0) cos(alpha0) d0;
%     0 0 0 1];
% 
% homo1_2 = [cos(theta2) -sin(theta2)*cos(alpha1) sin(theta2)*sin(alpha1) a1*cos(theta2);
%     sin(theta2) cos(theta2)*cos(alpha1) -cos(theta2)*sin(alpha1) a1*sin(theta2);
%     0 sin(alpha1) cos(alpha1) d1;
%     0 0 0 1];
% 
% homo2_3 = [cos(theta3) -sin(theta3)*cos(alpha2) sin(theta3)*sin(alpha2) a2*cos(theta3);
%     sin(theta3) cos(theta3)*cos(alpha2) -cos(theta3)*sin(alpha2) a2*sin(theta3);
%     0 sin(alpha2) cos(alpha2) d2;
%     0 0 0 1];
% 
% homo0_2 = homo0_1*homo1_2;
% homo0_3 = homo0_1*homo1_2*homo2_3;

homo0_3 = [ 7.5069398e-01 -6.1641818e-01  2.3767032e-01;
 -1.8368089e-01  1.5082608e-01  9.7134590e-01;
 -6.3460213e-01 -7.7283901e-01  6.1232343e-17];

homo0_2 = [ 8.6509484e-01 -4.4172809e-01  2.3767032e-01;
 -2.1167266e-01  1.0808267e-01  9.7134590e-01;
 -4.5475879e-01 -8.9061463e-01  6.1232343e-17];

homo0_1 = [-0.9713459  -0.23767032  0.        ;
  0.23767032 -0.9713459   0.        ;
  0.          0.          1.        ];

z1 = homo0_1([1 2 3],[3]);
%t10 = homo0_1([1 2 3],[4]);
z2 = homo0_2([1 2 3],[3]);
%t20 = homo0_2([1 2 3],[4]);
z3 = homo0_3([1 2 3],[3]);
%t30 = homo0_3([1 2 3],[4]);

tend = [-0.64058412; 0.26895448;  0.6554208];
t3 = [-0.36766532;  0.08996088;  0.28247249];
t2 = [0.;     0.;     0.0892];
t1 = [0.;     0.;     0.0892];

% z0 = [0;0;1];
% r0_0 = [1 0 0;0 1 0;0 0 1];
% r0_1 = homo0_1([1 2 3],[1,2,3]);
% r0_2 = homo0_2([1 2 3],[1,2,3]);

J = [z1 z2 z3; cross(z1,(tend-t1)) cross(z2,(tend-t2)) cross(z3,(tend-t3))]

%J2 = [r0_0*z0 r0_1*z0 r0_2*z0; cross(r0_0*z0,t3) cross(r0_1*z0,(t3-t1)) cross(r0_2*z0,(t3-t2))]
