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
%PD Constants
kp_1x=20;
kd_1x=4;
kp_2y=20;
kd_2y=4;
kp_3z=1020;
kd_3z=90;
kp_phi=990;
kd_phi=1;
kp_theta=990;
kd_theta=1;
kp_psi=990;
kd_psi=1;
phi_T=des_state.yaw;
r_des=des_state.yawdot;

%Errors
ex=des_state.pos(1)-state.pos(1);
ey=des_state.pos(2)-state.pos(2);
ez=des_state.pos(3)-state.pos(3);
ex_dot=des_state.vel(1)-state.vel(1);
ey_dot=des_state.vel(2)-state.vel(2);
ez_dot=des_state.vel(3)-state.vel(3);

%PD Controllers to calculate the accelerations in X,Y,Z, note that X=r1, Y=r2 and Z=r3
r1_ddot_des=des_state.acc(1)+kp_1x*ex+kd_1x*ex_dot;
r2_ddot_des=des_state.acc(2)+kp_2y*ey+kd_2y*ey_dot;
r3_ddot_des=des_state.acc(3)+kp_3z*ez+kd_3z*ez_dot;

%Position Control for 3D Trajectories Calculate the sum of forces(u1) using
% the acceleration in Z
u1=params.mass*params.gravity+params.mass*(r3_ddot_des);

%Attitude Control Calculate the Momentum matrix using
%the linearized model (that gives us the ecuations for phi_des, and theta des)
phi_des=(1/params.gravity)*(r1_ddot_des*sin(phi_T)-r2_ddot_des*cos(phi_T));
theta_des=(1/params.gravity)*(r1_ddot_des*cos(phi_T)+r2_ddot_des*sin(phi_T));
p_des=0;
q_des=0;
u2=[kp_phi*(phi_des-state.rot(1))+kd_phi*(p_des-state.omega(1));
    kp_theta*(theta_des-state.rot(2))+kd_theta*(q_des-state.omega(2));
    kp_psi*(des_state.yaw-state.rot(3))+kd_psi*(r_des-state.omega(3))];


% Thurst
F = u1;

% Moment
M = u2;

% =================== Your code ends here ===================

end
