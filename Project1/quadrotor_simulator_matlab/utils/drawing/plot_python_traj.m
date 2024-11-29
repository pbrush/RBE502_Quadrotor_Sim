function plot_python_traj(state_hist,state_des_hist,time_hist)
%PLOT_PYTHON_TRAJ Summary of this function goes here
%   Detailed explanation goes here
% Plot position for each quad
%state_des_hist = state_des_hist';
state_hist = state_hist';
h_pos = figure('Name', ['Quad: position']);
plot_state(h_pos, state_hist(1:3,:), time_hist, 'pos', 'sim');
plot_state(h_pos, state_des_hist(1:3,:), time_hist, 'pos', 'ref');
hLpos = legend({'sim', 'ref'},'Location','north','NumColumns',2);
newPosition = [0.68 0.92 0.2 0.02];
newUnits = 'normalized';
set(hLpos,'Position', newPosition,'Units', newUnits);
sgtitle('Position vs Time', 'Interpreter', 'LaTeX')
% Plot velocity for each quad
h_vel = figure('Name', ['Quad: velocity']);
plot_state(h_vel, state_hist(4:6,:), time_hist, 'vel', 'sim');
plot_state(h_vel, state_des_hist(4:6,:), time_hist, 'vel', 'ref');
hLvel = legend({'sim', 'ref'},'Location','north','NumColumns',2);
newPosition = [0.68 0.92 0.2 0.02];
set(hLvel,'Position', newPosition,'Units', newUnits);
sgtitle('Velocity vs Time', 'Interpreter', 'LaTeX')
end

