% function IK = InverseKinematics(leg, xd, yd, zd, yawd, pitchd, rolld)

% Approximate Analytical Solution

tic

sig = 1;

l0 = 2.21;
l1 = 2.8
lo1 = 0.25;
l2 = 2.9;
lo2 = 0.5;
l3 = 4.8;
l4 = 5.9;
lo5 = 0.5;

cy = cosd(yawd);     sy = sind(yawd);
cp = cosd(pitchd);   sp = sind(pitchd);
cr = cosd(rolld);    sr = sind(rolld);

O1 = [0; sig*l0; -l1];
F = [xd; yd; zd];

V1 = F - O1;
XF = [cy*cp; sy*cp; -sp];

Z2 = cross(V1, XF);
Z2 = -sig * Z2 / norm(Z2)

th1 = atan2(-sig * Z2(1), sig * Z2(2)) * 180/pi;
th2 = asin(-sig * Z2(3)) * 180/pi;

c1 = cosd(th1); s1 = sind(th1);
c2 = cosd(th2); s2 = sind(th2);

TB2 = [s1*s2, -sig*c1, -sig*s1*c2, sig*lo1*s1 + l2*c1 + lo2*s1*s2;
       -c1*s2, -sig*s1, sig*c1*c2, sig*l0 - sig*lo1*c1 + l2*s1 - lo2*c1*s2;
       -c2, 0, -sig*s2, -l1 - lo2*c2;
       0, 0, 0, 1];

V2 = inv(TB2)*[F; 1]; V2 = V2(1:3);

D = (norm(V2)^2 - l3^2 - l4^2) / (2*l3*l4);

th4 = atan2(-sig * sqrt(1 - D^2), D) * 180/pi;
c4 = cosd(th4); s4 = sind(th4);

beta = atan2(-V2(2), sqrt(V2(1)^2 + V2(3)^2)) * 180/pi;
alpha = atan2(l4*s4, l3 + l4*c4) * 180/pi;
th3 = beta - alpha;

th3 = -th3; th4 = -th4;

c3 = cosd(th3); s3 = sind(th3);
c34 = cosd(th3 + th4); s34 = sind(th3 + th4);

T24 = [c34, s34, 0, l3*c3 + l4*c34;
       s34, -c34, 0, l3*s3 + l4*s34;
       0, 0, -1, 0;
       0, 0, 0, 1];

TB4 = TB2*T24;

TBF = [cy*cp, -sy*cr + cy*sp*sr, sy*sr + cy*sp*cr, xd;
       sy*cp, cy*cr + sy*sp*sr, -cy*sr + sy*sp*cr, yd;
       -sp, cp*sr, cp*cr, zd;
       0, 0, 0, 1];

T4F = inv(TB4) * TBF;

th5 = atan2(-sig*T4F(1,1), sig*T4F(2,1)) * 180/pi;
th6 = atan2(sig*T4F(3,3), -sig*T4F(3,2)) * 180/pi;

% Numerical refining

thold = [th1; th2; th3; th4; th5; th6]

cont = 0;
tol = inf;

while tol > 1E-1

    % Direct Kinematics Error Calculation
    
    c1 = cosd(thold(1)); s1 = sind(thold(1));
    c2 = cosd(thold(2)); s2 = sind(thold(2));
    c3 = cosd(thold(3)); s3 = sind(thold(3));
    c4 = cosd(thold(4)); s4 = sind(thold(4));
    c5 = cosd(thold(5)); s5 = sind(thold(5));
    c6 = cosd(thold(6)); s6 = sind(thold(6));
    
    A0 = [0 -1 0 0; 1 0 0 sig*l0; 0 0 1 0; 0 0 0 1];
    A1 = [c1 0 -s1 -sig*lo1*c1; s1 0 c1 -sig*lo1*s1; 0 -1 0 -l1; 0 0 0 1];
    A2 = [-s2 0 sig*c2 -lo2*s2; c2 0 sig*s2 lo2*c2; 0 sig 0 -l2; 0 0 0 1];
    A3 = [c3 -s3 0 l3*c3; s3 c3 0 l3*s3; 0 0 1 0; 0 0 0 1];
    A4 = [c4 s4 0 l4*c4; s4 -c4 0 l4*s4; 0 0 -1 0; 0 0 0 1];
    A5 = [c5 0 -sig*s5 -lo5*c5; s5 0 sig*c5 -lo5*s5; 0 -sig 0 0; 0 0 0 1];
    A6 = [-s6 0 -c6 0; c6 0 -s6 0; 0 -1 0 0; 0 0 0 1];
    Af = [0 1 0 0; -1 0 0 0; 0 0 1 0; 0 0 0 1];
    
    TB0 = A0;       TB1 = TB0*A1;   TB2 = TB1*A2;   TB3 = TB2*A3;
    TB4 = TB3*A4;   TB5 = TB4*A5;   TB6 = TB5*A6;   TBF = TB6*Af;
    
    x = TBF(1,4); y = TBF(2,4); z = TBF(3,4);
    
    yaw = atan2(TBF(2,1), TBF(1,1)) * 180/pi;
    pitch = asin(-TBF(3,1)) * 180/pi;
    roll = atan2(TBF(3,2), TBF(3,3)) * 180/pi;
    
    delta_x = xd - x; delta_y = yd - y; delta_z = zd - z;
    delta_yaw = yawd - yaw; delta_pitch = pitchd - pitch; delta_roll = rolld - roll;
    delta_yaw = delta_yaw * pi/180;
    delta_pitch = delta_pitch * pi/180;
    delta_roll = delta_roll * pi/180;
    
    cy = cosd(yawd);     sy = sind(yawd);
    cp = cosd(pitchd);   sp = sind(pitchd);
    
    delta_thx = -sy*delta_pitch + cy*cp*delta_roll;
    delta_thy = cy*delta_pitch + sy*cp*delta_roll;
    delta_thz = delta_yaw - sp*delta_roll;
    
    e = [delta_x; delta_y; delta_z; delta_thx; delta_thy; delta_thz];

    % Jacobian Calculation
    
    Z0 = TB0(1:3,3);    Z1 = TB1(1:3,3);    Z2 = TB2(1:3,3);
    Z3 = TB3(1:3,3);    Z4 = TB4(1:3,3);    Z5 = TB5(1:3,3);
    
    O0 = TB0(1:3,4);    O1 = TB1(1:3,4);    O2 = TB2(1:3,4);
    O3 = TB3(1:3,4);    O4 = TB4(1:3,4);    O5 = TB5(1:3,4);
    O6 = TB6(1:3,4);
    
    J = [cross(Z0, O6 - O0) cross(Z1, O6 - O1) cross(Z2, O6 - O2) cross(Z3, O6 - O3) cross(Z4, O6 - O4) cross(Z5, O6 - O5); Z0 Z1 Z2 Z3 Z4 Z5];

    % Levenberg-Marquardt Method

    lambda = 0.001;

    C = J'*inv(J*J' + eye(6)*lambda^2);

    delta_th = C*e;
    thnew = thold + delta_th * 180/pi;
    
    cont = cont + 1;
    tol = norm(delta_th);
    
    [thold thnew]
        
    thold = thnew;
end

th = thold

cont
tol

toc