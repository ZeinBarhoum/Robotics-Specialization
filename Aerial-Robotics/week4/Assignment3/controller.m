function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
% Thrust
kp3 = 40;
kd3 = 5;


kpy = 1;
kdy  = 40;

kpf = 5;
kdf = 1;


kpywa = 40;
kdyaw = 5;



F = params.gravity * params.mass + params.mass*(kp3*(des_state.pos(3) - state.pos(3)) + kd3*(des_state.vel(3) - state.vel(3)));

ddr1_des = (des_state.acc(1) + kdy*(des_state.vel(1) - state.vel(1)) + kpy*(des_state.pos(1) - state.pos(1)));
ddr2_des = (des_state.acc(2) + kdy*(des_state.vel(2) - state.vel(2)) + kpy*(des_state.pos(2) - state.pos(2)));

phi_c = (1/params.gravity) * (ddr1_des*sin(des_state.yaw) - ddr2_des*cos(des_state.yaw));
theta_c = (1/params.gravity) * (ddr1_des*cos(des_state.yaw) + ddr2_des*sin(des_state.yaw));

dphi_c = 0;
dtheta_c = 0;

M =  [kpf*(phi_c - state.rot(1)) + kdf*(dphi_c - state.omega(1));
      kpf*(theta_c - state.rot(2)) + kdf*(dtheta_c - state.omega(2));
      kpywa*(des_state.yaw - state.rot(3)) + kdyaw*(des_state.yawdot - state.omega(3))];

% Moment
% M = zeros(3,1);
% 
% M =  [kpf*(phi_c - state.rot(1)) + kdf*(dphi_c - state.omega(1));
%       0;
%       0];

% =================== Your code ends here ===================

end
