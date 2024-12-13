function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;
persistent prevt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization
if ~isempty(t)
    desired_state = trajhandle(t, qn);
    prevt = 0;
end

% Inputs
F = 0;
M = [0;0;0];

% Physical Drone Parameters
m = params.mass;
I = params.I;
invI = params.invI;
g = params.grav;
L = params.arm_length;
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

% Current Params
cur_pos    = qd{qn}.pos;
cur_vel    = qd{qn}.vel;
cur_euler  = qd{qn}.euler;
cur_omega  = qd{qn}.omega;
dt = t - prevt;

phi   = cur_euler(1);
theta = cur_euler(2);
psi   = cur_euler(3);

% Concatenate into current state
cur_x = [ cur_pos;
          cur_vel;
          cur_euler;
          cur_omega ];

% Desired Params
desired_pos     = trajhandle(t).pos;
desired_vel     = trajhandle(t).vel;
desired_acc     = trajhandle(t).acc;
desired_jerk    = trajhandle(t).jerk;
desired_yaw     = trajhandle(t).yaw;
desired_yawdot  = trajhandle(t).yawdot;

% Transformation matrix from angular rates to pqr
transform = [cos(theta) 0 -cos(phi)*sin(theta);
                      0 1 0;
             sin(theta) 0 cos(phi)*cos(theta)];

% Get desired phi and theta
desired_roll   = (1 / g) * (desired_acc(1) * sin(desired_yaw) - desired_acc(2) * cos(desired_yaw));
desired_pitch  = (1 / g) * (desired_acc(1) * cos(desired_yaw) + desired_acc(2) * sin(desired_yaw));

% Get desired phi_dot an theta_dot
desired_phidt   = (1 / g) * ( (desired_jerk(1) * sin(desired_yaw) + desired_acc(1) * desired_yawdot * cos(desired_yaw) ) - ( desired_jerk(1) * cos(desired_yaw) - desired_acc(1) * desired_yawdot * sin(desired_yaw) ) );
desired_thetadt = (1 / g) * ( (desired_jerk(1) * cos(desired_yaw) - desired_acc(1) * desired_yawdot * sin(desired_yaw) ) + ( desired_jerk(1) * sin(desired_yaw) + desired_acc(1) * desired_yawdot * cos(desired_yaw) ) );
desired_angular_rates = [desired_phidt; desired_thetadt; desired_yawdot];

% Get desired pqr
desired_omega = transform * desired_angular_rates;

% Concatenate desired euler angles and desired angular rates
desired_euler = [ desired_roll; desired_pitch; desired_yaw ];

% Concatenate into desired state
desired_x = [ desired_pos;
              desired_vel;
              desired_euler;
              desired_omega ];

error_x = desired_x - cur_x;

r = cur_omega(1);
p = cur_omega(2);
q = cur_omega(3);

% % Dynamics
% A matrix
% A = [0, 0, 0, 1, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 1, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 1,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                  (981*sin(psi))/100,                                                                                                                (981*cos(psi))/100, (981*phi*cos(psi))/100 - (981*theta*sin(psi))/100,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                 -(981*cos(psi))/100,                                                                                                                (981*sin(psi))/100, (981*theta*cos(psi))/100 + (981*phi*sin(psi))/100,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                       (r*cos(theta))/(cos(theta)^2 + sin(theta)^2) - (p*sin(theta))/(cos(theta)^2 + sin(theta)^2),                                                 0,                    cos(theta)/(cos(theta)^2 + sin(theta)^2), 0,                   sin(theta)/(cos(theta)^2 + sin(theta)^2);
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 1,                                                          0;
%      0, 0, 0, 0, 0, 0, (r*cos(theta)*(sin(phi)*cos(theta)^2 + sin(phi)*sin(theta)^2))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2)^2 - (p*sin(theta)*(sin(phi)*cos(theta)^2 + sin(phi)*sin(theta)^2))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2)^2, - (p*cos(theta))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2) - (r*sin(theta))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2),                                                 0, -sin(theta)/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2), 0, cos(theta)/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2);
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0;
%      0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                   0,                                                                                                                                 0,                                                 0,                                                           0, 0,                                                          0];
% 
% B = [    0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%      100/3,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0,                                      0,                                      0,                                      0;
%          0, 590295810358705651712/8441230088129491,                                      0,                                      0;
%          0,                                      0, 590295810358705651712/8441230088129491,                                      0;
%          0,                                      0,                                      0, 295147905179352825856/8529774459683297];

desired_u = [1/m; 0; 0; 0];

[A, B] = get_A_B(desired_x, desired_u, params, dt);

% Q and R
Q = diag([10 10 10 1 1 1 1 1 1 1 1 1]);
R = diag([1 1 1 1]);

% Solve for the gains
[K, S] = lqr(A, B, Q, R);

% Apply gains to error to get inputs
u = -K * error_x;

% Decompose the inputs into F and M
F = u(1);
M = u(2:4);

% Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

disp(t)
prevt = t;

end
