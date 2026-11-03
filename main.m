function main()
%MAIN  Entry point: run both numerical examples of the paper.
%
%   Runs Case I (elliptical helix, 0-10 s) and Case II (upside-down
%   recovery, 0-6 s), generates Figures 4 & 5 plots and 3D animations.
%
%   Usage:  main
%
%   Requires MATLAB (R2018b or later recommended). Octave is NOT supported
%   because VideoWriter/print options are used.

addpath(genpath(pwd));

fprintf('============================================================\n');
fprintf(' Geometric Tracking Control of a Quadrotor UAV on SE(3)\n');
fprintf(' MATLAB reproduction of Lee, Leok & McClamroch, IEEE CDC 2010\n');
fprintf('============================================================\n\n');

run_case1();
fprintf('\n');
run_case2();

fprintf('\nDone. Results saved under results/case1_helix/ and results/case2_inverted_recovery/\n');

end
