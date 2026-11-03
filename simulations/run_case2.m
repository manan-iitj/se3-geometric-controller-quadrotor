function run_case2()
%SIMULATE_CASE2  Run Numerical Example II: upside-down recovery (0-6 s).
%
% Reproduces Figure 5 of the paper.

addpath(genpath(fileparts(mfilename('fullpath'))));

p   = quadrotor_parameters();
K   = controller_parameters(p);
casefun = @case2_trajectory;

% --- Initial conditions ----------------------------------------------------
x0  = [0; 0; 0];
v0  = [0; 0; 0];
R0  = [ 1,   0,      0;
        0, -0.9995, -0.0314;
        0,  0.0314, -0.9995 ];
Om0 = [0; 0; 0];
state0 = [x0; v0; R0(:); Om0];      % column-major R

tspan = [0 6];
t_eval = linspace(0, 6, 1001);

% --- Integrate -------------------------------------------------------------
opts = odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',0.01);
[t, state] = ode45(@(t,s) quadrotor_dynamics(t, s, p, K, casefun), ...
                   tspan, state0, opts);

% --- Extract signals -------------------------------------------------------
N = length(t);
x      = zeros(3,N); v      = zeros(3,N);
R_arr  = zeros(3,3,N); Om   = zeros(3,N);
xd_arr = zeros(3,N); xdd_arr= zeros(3,N);
ex_arr = zeros(3,N); ev_arr = zeros(3,N);
eR_arr = zeros(3,N); eOm_arr= zeros(3,N);
f_arr  = zeros(1,N); fr_arr = zeros(4,N);
Psi_arr= zeros(1,N); Rd_arr = zeros(3,3,N);
Om_d_arr = zeros(3,N);

for k = 1:N
    s = state(k,:)';                  % column state (15x1)
    x(:,k)   = s(1:3);
    v(:,k)   = s(4:6);
    R_arr(:,:,k) = reshape(s(7:15),3,3);
    Om(:,k)    = s(16:18);
    [f, M, dbg] = geometric_controller(t(k), s(1:3), s(4:6), ...
                                       reshape(s(7:15),3,3), s(16:18), ...
                                       p, K, casefun);
    xd_arr(:,k)   = dbg.xd;
    xdd_arr(:,k)  = dbg.xd_ddot;
    ex_arr(:,k)   = dbg.ex;
    ev_arr(:,k)   = dbg.ev;
    eR_arr(:,k)   = dbg.eR;
    eOm_arr(:,k)  = dbg.eOm;
    f_arr(k)      = dbg.f;
    fr_arr(:,k)   = dbg.f_rotors;
    Psi_arr(k)    = dbg.Psi;
    Rd_arr(:,:,k) = dbg.Rd;
    Om_d_arr(:,k) = dbg.Omega_d;
end

% --- Save data -------------------------------------------------------------
datadir = fullfile('results','case2_inverted_recovery','data');
plotdir = fullfile('results','case2_inverted_recovery','plots');
viddir  = fullfile('results','case2_inverted_recovery','videos');
if ~exist(datadir,'dir'), mkdir(datadir); end
if ~exist(plotdir,'dir'), mkdir(plotdir); end
if ~exist(viddir,'dir'), mkdir(viddir); end
save(fullfile(datadir,'case2_data.mat'), 't','x','v','R_arr','Om', ...
    'xd_arr','xdd_arr','ex_arr','ev_arr','eR_arr','eOm_arr', ...
    'f_arr','fr_arr','Psi_arr','Rd_arr','Om_d_arr','p','K');

% --- Numerical validation printout ----------------------------------------
fprintf('=== Case II: Upside-down recovery ===\n');
fprintf('Initial Psi  = %.4f   (paper: ~1.995)\n', Psi_arr(1));
fprintf('Initial attitude error [deg] = %.1f   (paper: ~178)\n', ...
        acosd((trace(R0'*eye(3))-1)/2));
idx_below1 = find(Psi_arr < 1, 1);
if ~isempty(idx_below1)
    fprintf('Psi < 1 at t = %.3f s   (paper: ~0.88 s)\n', t(idx_below1));
else
    fprintf('Psi never dropped below 1 in simulation window.\n');
end
fprintf('Final position error norm = %.4f m\n', norm(ex_arr(:,end)));
fprintf('Final angular velocity error norm = %.4f\n', norm(eOm_arr(:,end)));

% --- Plots ----------------------------------------------------------------
plot_case2_results(t, x, v, Om, xd_arr, ex_arr, eR_arr, eOm_arr, ...
                   f_arr, fr_arr, Psi_arr, R_arr, plotdir);

% --- Animation ------------------------------------------------------------
animate_case2(t, x, R_arr, viddir);

end
