function [F, M, trpy, drpy] = lqrcontroller(qd, t, qn, params, trajhandle)
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

if ~isempty(t)
    desired_state = trajhandle(t, qn);
end

%% Drone Parameters
Ixx  = params.I(1,1);
Iyy  = params.I(2,2);
Izz  = params.I(3,3);
g    = params.grav;
mass = params.mass;

% Create the current state
curx = [
    qd{qn}.pos;
    qd{qn}.vel;
    qd{qn}.euler;
    qd{qn}.omega
];

% Get relvavent current angles and angular rates
phi   = qd{qn}.euler(1);
theta = qd{qn}.euler(2);
psi   = qd{qn}.euler(3);
p     = qd{qn}.omega(1);
q     = qd{qn}.omega(2);
r     = qd{qn}.omega(3);

% Create the desired state
des_x = [desired_state.pos;
         desired_state.vel;
         0;                  % Assume desired phi is 0
         0;                  % Assume desired theta is 0
         desired_state.yaw;
         0;                  % Assume desired phidot is 0
         0;                  % Assume desired thetadot is 0
         desired_state.yawdot];

% Dynamics of the system
A = [0, 0, 0, 1, 0, 0,                                                         0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 1, 0,                                                          0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 1,                                                          0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                         (981*sin(psi))/100,                                     (981*cos(psi))/100, (981*phi*cos(psi))/100 - (981*theta*sin(psi))/100,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                        -(981*cos(psi))/100,                                     (981*sin(psi))/100, (981*theta*cos(psi))/100 + (981*phi*sin(psi))/100,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                                          0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                                          0,                         r*cos(theta) - p*sin(theta),                                                     0,                       cos(theta), 0,                        sin(theta);
    0, 0, 0, 0, 0, 0,            -(r*cos(theta) - p*sin(theta))/cos(phi)^2, (sin(phi)*(p*cos(theta) + r*sin(theta)))/cos(phi),                                                     0, (sin(phi)*sin(theta))/cos(phi), 1, -(cos(theta)*sin(phi))/cos(phi);
    0, 0, 0, 0, 0, 0, (sin(phi)*(r*cos(theta) - p*sin(theta)))/cos(phi)^2,            -(p*cos(theta) + r*sin(theta))/cos(phi),                                                     0,            -sin(theta)/cos(phi), 0,              cos(theta)/cos(phi);
    0, 0, 0, 0, 0, 0,                                                         0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                                         0,                                                       0,                                                     0,                                 0, 0,                                  0;
    0, 0, 0, 0, 0, 0,                                                         0,                                                       0,                                                     0,                                 0, 0,                                  0];
 

B = [     0     0     0     0;
          0     0     0     0;
          0     0     0     0;
          0     0     0     0;
          0     0     0     0;
     1/mass     0     0     0;
          0     0     0     0;
          0     0     0     0;
          0     0     0     0;
          0 1/Ixx     0     0;
          0     0 1/Iyy     0;
          0     0     0 1/Izz ];


% Ceate Q and R
Q = diag([10,10,10,0.1,0.1,0.1,0.01,0.01,0.01,0.01,0.01,0.01]);
R = diag([0.01,0.1,0.1,0.1]);

% Compute gain:
[K, S] = lqr(A, B, Q, R);

% Get error and apply gain to it
state_error = curx - des_x;
eu          = -K * state_error;

des_u = [mass*g; 0; 0; 0];

u = des_u + eu;

% Extract thrust and moments from u
F = u(1);
M = u(2:4);

% Set trpy and drpy (usually for hardware outputs)
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
