function PV_f = PV_Profile(T,t0)

% Get current Simulink time
% t0 = getCurrentTime(); 

% Simulation timestep
dt = 3600; 

% Preallocate forecast vector
PV_f = zeros(T,1);

% PV sinusoidal model parameters
Pmax = 1200;       % peak PV power
day_length = 86400; % seconds per day (24 hours)

% Generate forecast for next T steps
for k = 1:T
    t_future = t0 + k*dt;

    % Sinusoidal PV model (simple example)
    % PV(t) = max(Pmax * sin(pi * t / day_length), 0)
    PV_f(k) = max(Pmax * sin(pi * t_future / day_length), 0);
end
