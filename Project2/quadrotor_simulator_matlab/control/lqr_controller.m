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
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization
if ~isempty(t)
    desired_state = trajhandle(t, qn);
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

% Get desired phi and theta
desired_roll   = (1 / g) * (desired_acc(1) * sin(desired_yaw) - desired_acc(2) * cos(desired_yaw));
desired_pitch  = (1 / g) * (desired_acc(1) * cos(desired_yaw) + desired_acc(2) * sin(desired_yaw));

% Get desired r_dot and theta_dot
desired_rolldot   = (1 / g) * ( (desired_jerk(1) * sin(desired_yaw) + desired_acc(1) * desired_yawdot * cos(desired_yaw) ) - ( desired_jerk(1) * cos(desired_yaw) - desired_acc(1) * desired_yawdot * sin(desired_yaw) ) );
desired_pitchdot  = (1 / g) * ( (desired_jerk(1) * cos(desired_yaw) - desired_acc(1) * desired_yawdot * sin(desired_yaw) ) + ( desired_jerk(1) * sin(desired_yaw) + desired_acc(1) * desired_yawdot * cos(desired_yaw) ) );

% Concatenate desired euler angles and desired angular rates
desired_euler = [ desired_roll; desired_pitch; desired_yaw ];
desired_omega = [ desired_rolldot; desired_pitchdot; desired_yawdot ];

% Concatenate into desired state
desired_x = [ desired_pos;
              desired_vel;
              desired_euler;
              desired_omega ];

error_x = desired_x - cur_x;

% Horizon
N = 10;

% Dynamics
A = zeros(12,12);
B = zeros(12,4);

%    x y z xdt ydt zdt phi the psi   p   q   r
A = [0 0 0   1   0   0   0   0   0   0   0   0;     % xdt
     0 0 0   0   1   0   0   0   0   0   0   0;     % ydt
     0 0 0   0   0   1   0   0   0   0   0   0;     % zdt
     0 0 0   0   0   0   0   0   0   0   0   0;     % xddt
     0 0 0   0   0   0   0   0   0   0   0   0;     % yddt
     0 0 0   0   0   0   0   0   0   0   0   0;     % zddt
     0 0 0   0   0   0   0   0   0   1   0   0;     % phidt
     0 0 0   0   0   0   0   0   0   0   1   0;     % thetadt
     0 0 0   0   0   0   0   0   0   0   0   1;     % psidt
     0 0 0   0   0   0   0   0   0   0   0   0;     % pdt
     0 0 0   0   0   0   0   0   0   0   0   0;     % qdt
     0 0 0   0   0   0   0   0   0   0   0   0;];   % rdt

%      F     M1     M2     M3
B = [  0      0      0      0;     % xdt
       0      0      0      0;     % ydt
     1/m      0      0      0;     % zdt
       0      0      0      0;     % xddt
       0      0      0      0;     % yddt
       0      0      0      0;     % zddt
       0      0      0      0;     % phi   = p?
       0      0      0      0;     % theta = q?
       0      0      0      0;     % psi   = r?
       0  1/Ixx      0      0;     % pdt
       0      0  1/Iyy      0;     % qdt
       0      0      0  1/Izz;];   % rdt

% Q and R
Q = eye(12);
R = eye(4);

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

end
