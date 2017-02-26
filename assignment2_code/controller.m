function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%20/20 in line trayectory with
%Cumulative position error: 0.0065091
%30/30 in sine wave trayectory with
%Cumulative position error: 0.092863
%Kp_z=1020;Kd_z=90;Kp_phi=990;Kd_phi=1;Kp_y=20;Kd_y=4;  
%Final time: 3.9 sec


%PD Constants
Kp_z=1020;
Kd_z=90;
Kp_phi=990;
Kd_phi=1;
Kp_y=20;
Kd_y=4;

%Errors
ey=des_state.pos(1)-state.pos(1);
ez=des_state.pos(2)-state.pos(2);
ey_dot=des_state.vel(1)-state.vel(1);
ez_dot=des_state.vel(2)-state.vel(2);
phic=(-1/params.gravity)*(des_state.acc(1)+Kd_y*ey_dot+Kp_y*ey);
phic_dot=(-1/params.gravity)*(Kd_y*(des_state.acc(1)+(params.gravity*state.omega(1)))+Kp_y*ey_dot);
ephi=phic-state.rot(1);
ephi_dot=phic_dot-state.omega(1);
%Control Signals
u1=params.mass*(params.gravity+des_state.acc(2)+Kd_z*ez_dot+Kp_z*ez);
u2=Kp_phi*ephi+Kd_phi*ephi_dot;
% FILL IN YOUR CODE HERE

end

