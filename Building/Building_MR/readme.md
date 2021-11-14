This file presents the instructions to simulate the DoS attacks on the 20 story building with MR dampers.
Matlab from 2021a or newer is required for this operation. The following toolboxes are required:
- Parallel toolbox
- Global optimization toolbox
- Control system toolbox

The steps to run the simulation are the ones that follow:

1. Modify the relevant parameters in the parameters section on the "B20_MR_DoS_design.m" script. Ecase sets the earthquake to be used, and EI sets the
scaling done to the signal. Change DoS_s from ones(20,1) to zeros(20,1) if only the passive component has to be analyzed.
2. Run the script "B20_MR_DoS_design.m". Note that it takes a long time due to the amount of simulations and optimizations performed. The plotting
functions "DoS_plot.m" and "DoS_comp_plot.m" are called in this script, so there is no need to call them separately.
3. In case the user wants to see disabled actuators per attack, run the "Attacked_per_selection.m" script. Here, the visualization of the disabled floors
per attack can be seen, along with a time series of the top floor ISD.

Important notes:
- The LQR controller can be tuned further by changing the r_lqr and q_lqr parameters.
- Since the attacks are designed via a genetic algorithm, the results may vary from those presented in the paper, since this algorithm guarantees a local
maximum and not necessarily a global maximum. The results presented in the paper were determined to be the global maximums after running the script 
several times.
- Under the "genetic algorithm" section of the code, the parameters for the optimization problems can be tuned. the variable "opts" presents the parameters
to be changed, like the use of the parallel toolbox and the maximum generations.
- The "Comparison of different attacks with k=5" compares different attacks with the optimal with k=5. The results obtained in every simulation may vary,
since one of the attacks is random.