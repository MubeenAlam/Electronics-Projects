% angles in MATLAB are in rad
syms thi1 thi2 thi3
syms Px Py Pz
d2 = 0.1491;
a3 = -0.0203;
a2 = 0.4318;
d4 = 0.43307;
d6 = 0.05625;
P = [Px,Py,Pz];
ore = [-0.1491,d4+d6+a2,a3];
ARM = -1;
ELBOW = -1;
theta1 = atan2(-ARM*Py*sqrt(Px^2+Py^2-d2^2)-Px*d2 , -ARM*Px*sqrt(Px^2+Py^2-d2^2)+Py*d2);
t1 = vpa(subs(theta1,Px,ore(1)),3);
t1 = vpa(subs(t1,Py,ore(2)),3);

alpha = acos(-ARM*sqrt(Px^2+Py^2-d2^2) / sqrt(Px^2+Py^2+Pz^2-d2^2));
beta = acos( (a2^2+Px^2+Py^2+Pz^2-d2^2-(d4^2+a3^2)) / (2*a2*sqrt(Px^2+Py^2+Pz^2-d2^2)));
theta2 = atan2( sin(alpha+ARM*ELBOW*beta) , cos(alpha+ARM*ELBOW*beta)  );

t2 = vpa(subs(theta2,Px,ore(1)),3);
t2 = vpa(subs(t2,Py,ore(2)),3);
t2 = vpa(subs(t2,Pz,ore(3)),3);

R = sqrt(Px^2+Py^2+Pz^2-d2^2);
cos_phi = (a2^2+(d2^2+a3^2)- R^2) / (2*a2*sqrt(d4^2+a3^2));
sin_phi = ARM*ELBOW*sqrt(1-cos_phi^2 );
sin_beta = d4/sqrt(d4^2+a3^2);
cos_beta = abs(a3)/sqrt(d4^2+a3^2);
theta3 = atan2(sin_phi*cos_beta-cos_phi*sin_beta , cos_phi*cos_beta+sin_phi*sin_beta);


t3 = vpa(subs(theta3,Px,ore(1)),3);
t3 = vpa(subs(t3,Py,ore(2)),3);
t3 = vpa(subs(t3,Pz,ore(3)),3);






