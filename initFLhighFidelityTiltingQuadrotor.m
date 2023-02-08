run highFidelityTiltingQuadrotorData

load benchmark_trajectories/b01.mat reference_FL tt Ts_st

Ts = 1e-3;

ref_FL.time = tt';
ref_FL.signals.dimensions = 25;
ref_FL.signals.values = reference_FL;%% Controller params

FL.K_p1 = 28.5*eye(3);
FL.K_p2 = 271*eye(3);
FL.K_p3 = 857*eye(3);

FL.K_o1 = 45*eye(3);
FL.K_o2 = 675*eye(3);
FL.K_o3 = 3375*eye(3);

w_bar2_min = (0.01*prop.w_max).^2;
w_bar2_rest = tiltq.w_bar_hover_planar.^2;

kh1 = 1;
kh2 = 2;

% add optional model uncertainties
run addModelUncertainties.m

tiltq.m_b = tiltq.m_b .* ( 1+modelRelativeUncertanties.mb );
body.I = body.I .* ( 1+modelRelativeUncertanties.Ib );
prop.k_m = prop.k_m .* ( 1+modelRelativeUncertanties.km );
prop.k_t = prop.k_t .* ( 1+modelRelativeUncertanties.kt );