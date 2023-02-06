% Initialization data for the hight fidelity tilting quadrotor model

skew = @(x) [
    0 -x(3) x(2)
    x(3) 0 -x(1)
    -x(2) x(1) 0
    ];

tiltq.N = 4;

% propeller data
prop.m = 5e-3;      % propeller mass [kg]
prop.R = 80e-3;     % propeller Radius [m]
prop.h = 10e-3;     % propeller height [m]
% propeller inertia matrix [kg*m^2]
prop.I = diag([ ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/12*prop.m*(3*prop.R^2+prop.h^2) ...
    1/2*prop.m*prop.R^2]);

rotor.m = 50e-3;    % rotor mass [kg]
rotor.R = 20e-3;    % rotor Radius [m]
rotor.h = 10e-3;    % rotor height [m]
% rotor inertia matrix [kg*m^2]
rotor.I = diag([ ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/12*rotor.m*(3*rotor.R^2+rotor.h^2) ...
    1/2*rotor.m*rotor.R^2]);

tiltq.I_i = prop.I+rotor.I;   % total rotor+propeller inertia [Kg*m^2]

stator.m = 100e-3;      % stator mass [kg]

% main body data
body.m = 2;             % body mass [kg]
body.R = 100e-3;        % body Radius [m]
body.h = 50e-3;         % body height [m]
body.I = diag([ ...        % body inertia [kg*m^2]
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/12*body.m*(3*body.R^2+body.h^2) ...
    1/2*body.m*body.R^2]);
tiltq.m_b = body.m + tiltq.N*(stator.m+rotor.m+prop.m);       % body mass [Kg]

prop.w_max_rpm = 10000;                 % propeller maximum rate [rpm]
prop.w_max = prop.w_max_rpm * 2*pi/60;  % propeller maximum rate [rad/s]
prop.w_min = sqrt(0.01*prop.w_max.^2);

prop.max_thrust = 3*tiltq.m_b*9.81/4;         % propeller maximum thrust [N]           

prop.k_t = prop.max_thrust / prop.w_max.^2;  % thrust coefficient [N/s^2]
prop.m_t_ratio = 0.05;
prop.k_m = prop.k_t * prop.m_t_ratio;                       % propeller drag coefficient [Nm/s^2]

% motor torque needed to reach the maximum speed.
% it has to counter the drag torque at maximum speed
prop.max_drag = prop.k_m * prop.w_max.^2;

% propeller spinning convention
% 1 if positive velocity (CCW) gives positive thrust
% -1 if negative velocity (CW) gives positive thrust
prop.c_direction = [1 1 -1 -1];

% rotor position in body frame [m]
%   3(CW)   front    1(CCW)
%   left      +      right
%   2(CCW)   back    4(CW)
tiltq.p_mot = ...         % rotor position w.r.t. body
    [ 0.2 -0.2 0;
     -0.2  0.2 0;
      0.2  0.2 0;
     -0.2 -0.2 0];
tiltq.theta = atan2(tiltq.p_mot(:,2),tiltq.p_mot(:,1));   % arm direction angle
tiltq.beta = tiltq.theta;       % propellers tilting plane yaw angle [rad]

% final body inertia when the masses of motors and propellers are
% considered
for k=1:4
    body.I = body.I - (stator.m+rotor.m+prop.m)*skew(tiltq.p_mot(k,:))^2;
end

% HOVER DATA
tiltq.required_hover_thrust = tiltq.m_b * 9.81;
tiltq.w_bar_hover_planar = sqrt(tiltq.required_hover_thrust/4/prop.k_t);

% planar control allocation matrix
% torque allocation matrix
tiltq.M = cross(tiltq.p_mot.',kron([0;0;1],prop.c_direction)) * prop.k_t +...
    kron([0;0;1],[-1 -1 -1 -1])*prop.k_m;
% thrust allocation matrix
tiltq.T = kron([0;0;1],prop.c_direction)*prop.k_t;

tiltq.A = [tiltq.T(3,:); tiltq.M];
tiltq.Alloc = inv(tiltq.A);


%% constraints

tiltq.max_alpha         = pi/3 .* [1 1 1 1];        % maximum tilting angles [rad]
tiltq.min_alpha         = -pi/3 .* [1 1 1 1];       % minimum tilting angles [rad]
tiltq.max_alpha_dot     = 6 .* [1 1 1 1];           % maximum tilting rates [rad/s]
tiltq.min_alpha_dot     = -6 .* [1 1 1 1];          % minimum tilting rates [rad/s]

%% Initial conditions

% initial position [m]
p_0 = [
    0
    0
    0
    ];
% initial quaternion
q_0 = [
    1
    0
    0
    0
    ];
% initial linear velocity (world frame) [m/s]
v_0 = [
    0
    0
    0
    ];
% initial angular velocity (body frame) [rad/s]
omega_0 = [
    0
    0
    0
    ];
% initial tilting angles [rad]
alpha_0 = [
    0
    0
    0
    0
    ];
alpha_0 = alpha_0(1:tiltq.N);
% initial rate of the tilting angles [rad/s]
alpha_dot_0 = [
    0
    0
    0
    0
    ];
alpha_dot_0 = alpha_dot_0(1:tiltq.N);
% initial propellers spinning rate [rad/s]
w_bar_0 = [
    tiltq.w_bar_hover_planar
    tiltq.w_bar_hover_planar
    -(tiltq.w_bar_hover_planar)
    -(tiltq.w_bar_hover_planar)
    ];
w_bar_0 = w_bar_0(1:tiltq.N);

% initial state vector
x_0 = [
    p_0
    q_0
    v_0
    omega_0
    alpha_0
    alpha_dot_0
    w_bar_0
    ];

% step size for the quadrotor
T_s = 1e-3;