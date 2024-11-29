function [desired_state,n] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
Radius = 1;
%zinitial = [0 Radius/2 0 0 0 0]';
zinitial = [0 0 0 0 Radius/2 0 0 0]';
%thetainitial = [0 2*pi 0 0 0 0]';
thetainitial = [0 0 0 0 2*pi 0 0 0]';
t0 = 0;
tf = 8;

if t<=tf
    n = 1; 
else 
    n = 2;
end

A = [generate_poly(7,3,t0);
     generate_poly(7,3,tf)];
 
%A = [1  t0 t0^2 t0^3   t0^4   t0^5;
%     1  tf tf^2 tf^3   tf^4   tf^5;
%     0  1  2*t0 3*t0^2 4*t0^3 5*t0^4;
%     0  1  2*tf 3*tf^2 4*tf^3 5*tf^4;
%     0  0  2    6*t0   12*t0^2 20*t0^3;
%     0  0  2    6*tf   12*tf^2 20*tf^3;];
 
zcoeff = inv(A)*zinitial;
thetacoeff = inv(A)*thetainitial;
%polynominalmat = [1 t  (t)^2   (t)^3   (t)^4   (t)^5;
%                  0 1  2*(t)   3*(t)^2 4*(t)^3 5*(t)^4;
%                  0 0  2       6*(t)   12*(t)^2 20*(t)^3];
polynominalmat = generate_poly(7,3,t);
theta_d = polynominalmat*thetacoeff;
z_d = polynominalmat*zcoeff;

x_pos = Radius * cos(theta_d(1));
y_pos = Radius * sin(theta_d(1));
x_vel = -Radius * sin(theta_d(1)) * theta_d(2);
y_vel =  Radius * cos(theta_d(1)) * theta_d(2);
x_acc = -Radius * cos(theta_d(1)) * theta_d(2)^2 - Radius * sin(theta_d(1)) * theta_d(3);
y_acc = -Radius * sin(theta_d(1)) * theta_d(2)^2 + Radius * cos(theta_d(1)) * theta_d(3);
x_jrk = Radius * sin(theta_d(1)) * theta_d(2)^3 -3*Radius * cos(theta_d(1)) * theta_d(2) * theta_d(3) - Radius * sin(theta_d(1)) * theta_d(4);
y_jrk = -Radius * cos(theta_d(1)) * theta_d(2)^3 + Radius * cos(theta_d(1)) * theta_d(3);

switch n 
    
    case 1
        pos = [x_pos ; y_pos; z_d(1)];
        vel = [x_vel ; y_vel; z_d(2)];
        acc = [x_acc ; y_acc; z_d(3)];
        
    case 2 
        pos = [1 ; 0; 0.5];
        vel = [0 ; 0; 0];
        acc = [0 ; 0; 0]; 
        
end

yaw = 0;
yawdot = 0;



% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.jerk = [0;0;0];
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
