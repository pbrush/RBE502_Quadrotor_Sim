function [desired_state] = jump(t, qn, map, path)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

persistent goal_pos;

if nargin ~= 2
    goal_pos = path{end};
else
    if t<=1
        pos = [0 ; 0; 0];
        vel = [0 ; 0; 0];
        acc = [0 ; 0; 0];
    elseif t>1 && t<=2
        pos = goal_pos;
        vel = [0 ; 0; 0];
        acc = [0 ; 0; 0];
    elseif t>2
        pos = goal_pos;
        vel = [0 ; 0; 0];
        acc = [0 ; 0; 0];
    end
    yaw = 0;
    yawdot = 0;
    
% =================== Your code ends here ===================
    
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
end
end
