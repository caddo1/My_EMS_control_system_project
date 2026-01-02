function [u0, e0, reward0] = pv_battery_ems(PV_forecast, SoC0, loadP, R, T)

nLoads = length(loadP);

% Decision variables
u       = sdpvar(T,1);                 % net battery power
u_pos   = sdpvar(T,1);                 % charging component (>=0)
u_neg   = sdpvar(T,1);                 % discharging component (>=0)
e       = binvar(nLoads, T);           % load on/off
SoC     = sdpvar(T+1,1);               % SoC trajectory

Constraints = [SoC(1) == SoC0];

% Parameters
SoC_min = 0.2; 
SoC_max = 0.9;

u_min   = -2000;
u_max   =  2000;

P_nom = 50000;      % Wh
eta_c = 0.98;
eta_d = 0.98;

E_nom = 3600 * P_nom;
dt    = 3600;

alpha_SoC = 0.1;
SoC_ref   = 0.5;

Objective = 0;

for k = 1:T

    % Load demand
    P_req = loadP' * e(:,k);

    % Net supply
    P_sup = PV_forecast(k) + u(k);

    % === Objective: ONLY reward (no mismatch penalty) ===
    Objective = Objective - (R' * e(:,k));

    % === Linear battery decomposition ===
    % u = u_pos - u_neg, both >= 0
    Constraints = [Constraints;
        u(k) == u_pos(k) - u_neg(k);
        u_pos(k) >= 0;
        u_neg(k) >= 0;
        u(k) >= u_min;
        u(k) <= u_max;
    ];

    % === PHYSICAL POWER BALANCE (no grid) ===
    Constraints = [Constraints;
        PV_forecast(k) + u_neg(k) >= P_req + u_pos(k);
        u_neg(k) <= P_req;
    ];
    % === Linear SoC update ===
    Constraints = [Constraints;
        SoC(k+1) == SoC(k) + dt/E_nom*(eta_c * u_pos(k) - (1/eta_d) * u_neg(k));
        SoC_min <= SoC(k+1) <= SoC_max;
    ];
end

% Terminal SoC penalty
Objective = Objective + alpha_SoC * (SoC(T+1) - SoC_ref)^2;

% Use Gurobi (preferred) or any MILP solver
ops = sdpsettings('solver','bnb','verbose',0);

sol = optimize(Constraints, Objective, ops);

if sol.problem ~= 0
    warning('Optimization failed. Using fallback.');
    u0 = 0; e0 = zeros(nLoads,1); reward0 = 0;
    return;
end

% First-step outputs
u0 = value(u(1));
e0 = value(e(:,1));
reward0 = R' * e0;   % no mismatch cost anymore

end
