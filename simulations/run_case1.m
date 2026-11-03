function run_case1()
%SIMULATE_CASE1  Run Numerical Example I: elliptical helix (0-10 s).
%
% Reproduces Figure 4 of the paper.

addpath(genpath(fileparts(mfilename('fullpath'))));

p   = quadrotor_parameters();
K   = controller_parameters(p);
casefun = @case1_trajectory;

% --- Initial conditions ----------------------------------------------------
x0  = [0; 0; 0];
v0  = [0; 0; 0];
R0  = eye(3);
Om0 = [0; 0; 0];
state0 = [x0; v0; R0(:); Om0];      % column-major R

tspan = [0 10];
t_eval = linspace(0, 10, 1001);     % output times

% --- Integrate (ode45, tight tolerances) ----------------------------------
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

persistent_db = [];

for k = 1:N
    s = state(k,:)';                  % column state (15x1)
    x(:,k)   = s(1:3);
    v(:,k)   = s(4:6);
    R_arr(:,:,k) = reshape(s(7:15),3,3);
    Om(:,k)    = s(16:18);
    % Re-run controller at this state to log debug outputs
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
datadir = fullfile('results','case1_helix','data');
plotdir = fullfile('results','case1_helix','plots');
viddir  = fullfile('results','case1_helix','videos');
if ~exist(datadir,'dir'), mkdir(datadir); end
if ~exist(plotdir,'dir'), mkdir(plotdir); end
if ~exist(viddir,'dir'), mkdir(viddir); end
save(fullfile(datadir,'case1_data.mat'), 't','x','v','R_arr','Om', ...
    'xd_arr','xdd_arr','ex_arr','ev_arr','eR_arr','eOm_arr', ...
    'f_arr','fr_arr','Psi_arr','Rd_arr','Om_d_arr','p','K');

% --- Numerical validation printout ----------------------------------------
fprintf('=== Case I: Elliptical helix ===\n');
fprintf('Initial Psi  = %.4f   (from initial conditions)\n', Psi_arr(1));
fprintf('Final   Psi  = %.4f\n', Psi_arr(end));
fprintf('Max position error norm = %.4f m\n', max(vecnorm(ex_arr)));
fprintf('Final position error norm = %.4f m\n', norm(ex_arr(:,end)));
fprintf('Max angular velocity error norm = %.4f\n', max(vecnorm(eOm_arr)));

% --- Plots ----------------------------------------------------------------
plot_case1_results(t, x, v, Om, xd_arr, ex_arr, eR_arr, eOm_arr, ...
                   f_arr, fr_arr, Psi_arr, R_arr, plotdir);

% --- Animation ------------------------------------------------------------
animate_case1(t, x, R_arr, xd_arr, viddir);

end
