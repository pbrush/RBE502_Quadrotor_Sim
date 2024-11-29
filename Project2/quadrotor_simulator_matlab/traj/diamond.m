function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
unit = sqrt(2);
unit1 = 1;
xinitial = [0 unit1 0 0 0 0]';
yinitial = [0 unit 0 0 0 0]';
%zinitial = [0 length 0 0 0 0]';
t0 = 0;
tfinal = 8;
tf = tfinal/4;
A = [1  t0 t0^2 t0^3   t0^4   t0^5;
     1  tf tf^2 tf^3   tf^4   tf^5;
     0  1  2*t0 3*t0^2 4*t0^3 5*t0^4;
     0  1  2*tf 3*tf^2 4*tf^3 5*tf^4;
     0  0  2    6*t0   12*t0^2 20*t0^3;
     0  0  2    6*tf   12*tf^2 20*tf^3;];
 
B = [1  t0 t0^2 t0^3   t0^4   t0^5;
     1  tfinal tfinal^2 tfinal^3   tfinal^4   tfinal^5;
     0  1  2*t0 3*t0^2 4*t0^3 5*t0^4;
     0  1  2*tfinal 3*tfinal^2 4*tfinal^3 5*tfinal^4;
     0  0  2    6*t0   12*t0^2 20*t0^3;
     0  0  2    6*tfinal   12*tfinal^2 20*tfinal^3;];
 
xcoeff = inv(B)*xinitial;
ycoeff = inv(A)*yinitial;

if t <= tf 
    n=0;
elseif t>tf && t<=2*tf 
    n=1;
elseif t>2*tf && t<=3*tf 
    n=2;
elseif t>3*tf && t<=4*tf 
    n=3;
else
    n=4;
end
    
polynominalmatyz = [1 t-n*tf (t-n*tf)^2 (t-n*tf)^3 (t-n*tf)^4 (t-n*tf)^5;
                  0 1 2*(t-n*tf) 3*(t-n*tf)^2 4*(t-n*tf)^3 5*(t-n*tf)^4;
                  0 0 2 6*(t-n*tf) 12*(t-n*tf)^2 20*(t-n*tf)^3];
              
polynominalmat = [1 t  (t)^2   (t)^3   (t)^4   (t)^5;
                  0 1  2*(t)   3*(t)^2 4*(t)^3 5*(t)^4;
                  0 0  2       6*(t)   12*(t)^2 20*(t)^3];
              
x_d = polynominalmat * xcoeff;
yz_d = polynominalmatyz * ycoeff;

switch n 
case 0
    y_d = yz_d;
    z_d = yz_d;
case 1 
    y_d = [unit 0 0]' - yz_d;
    z_d = [unit 0 0]' + yz_d;
case 2
    y_d = -yz_d;
    z_d = [2*unit 0 0]' - yz_d;
case 3
    y_d = [-unit 0 0]' + yz_d;
    z_d = [unit 0 0]' - yz_d;
case 4
    x_d = [1 0 0]';
    y_d = [0 0 0]';
    z_d = [0 0 0]';
end

pos = [x_d(1) ; y_d(1); z_d(1)];
vel = [x_d(2) ; y_d(2); z_d(2)];
acc = [x_d(3) ; y_d(3); z_d(3)];

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
