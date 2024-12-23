function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
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

 persistent pitch_err;
 persistent roll_err;
 persistent pitchdot_err;
 persistent rolldot_err;

 % Roll and Pitch at 100hz
 % Yaw at 500hz

 
% =================== Your code starts here ===================
%% Parameter Initialization

% Current Params
cur_pos = qd{qn}.pos;
cur_vel = qd{qn}.vel;
cur_euler = qd{qn}.euler;
cur_omega = qd{qn}.omega;

% Attitude Control Gains
kp_att = [150, 150, 125]';
kd_att = [75, 75, 85]';

% Position Control gains
kp_pos = [8.2, 7, 15]';%[4, 4, 3];
kd_pos = [5.3, 6, 8]';%[7, 7, 5];

% Custom Initial Params
traj_rolldot = 0;
traj_pitchdot = 0;

% Traj Params
traj_pos = qd{qn}.pos_des;
traj_vel = qd{qn}.vel_des;
traj_acc = qd{qn}.acc_des;
traj_yaw = qd{qn}.yaw_des;
traj_yawdot = qd{qn}.yawdot_des;

% Physical Drone Parameters
m = params.mass;
I = params.I;
invI = params.invI;
g = params.grav;
L = params.arm_length;

% Dynamic Constraints
max_ang = params.maxangle;
max_F = params.maxF;
min_F = params.minF;

%% Equations

% Calc pose and vel errors
pos_err = traj_pos - cur_pos;
vel_err = traj_vel - cur_vel;

% Get acceleration
des_acc = traj_acc + kp_pos .* pos_err + kd_pos .* vel_err; % kp_pos' * pos_err' - kd_pos' * cur_vel'; % might have acc_des and vel_err

% Get desired phi and theta
traj_roll = (1 / g) * (des_acc(1) * sin(traj_yaw) - des_acc(2) * cos(traj_yaw));
traj_pitch = (1 / g) * (des_acc(1) * cos(traj_yaw) + des_acc(2) * sin(traj_yaw));

% Get angular errors
roll_err = wrapToPi(traj_roll - cur_euler(1));
pitch_err = wrapToPi(traj_pitch - cur_euler(2));
yaw_err = wrapToPi(traj_yaw - cur_euler(3));

% Get p, q, and r
intermediary_mat = [cos(cur_euler(2)) 0 -cos(cur_euler(1))*sin(cur_euler(2));
                            0         1            sin(cur_euler(1))        ;
                    sin(cur_euler(2)) 0  cos(cur_euler(1))*cos(cur_euler(2));];

pqr = intermediary_mat * [cur_omega(1); cur_omega(2); cur_omega(3)];

% Get angular rate errors
p_err = traj_rolldot - cur_omega(1);
q_err = traj_pitchdot - cur_omega(2);
r_err = traj_yawdot - cur_omega(3);

% disp("**************")
% disp(roll_err)
% disp(pitch_err)
% disp(yaw_err)
% disp(t)

% Get U1 and U2
u1 = m * ( g + des_acc(3) );
u2 = I * [kp_att(1) * roll_err  + kd_att(1) * p_err;
          kp_att(2) * pitch_err + kd_att(2) * q_err;
          kp_att(3) * yaw_err   + kd_att(3) * r_err;];

% Assign Outputs
if u1 > params.maxF
    u1 = params.maxF;
elseif u1 < params.minF
    u1 = params.minF;
end

F = u1;
M = u2;

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];    % Standard RPY
drpy = [0, 0, 0, 0];    % Derivativees RPY

end
