This file presents the instructions to simulate both the DoS and FDI attacks on the 20 story building with AMD and ATMD.
Matlab from 2021a or newer is required for this operation. The Control system toolbox is needed to run these simulations

DoS attacks:
1. Clear the workspace by typing "clear all" in the command Window.
2. From the "B20_AMD_main.m" file, change the parameters needed. The FDIon variable should be set to 0. The Ecase variable
toggles the earthquake for the simulation, and EI changes the intensity. Note that the value of EI should be changed only
on the instance that was specified, the other one changes its value depending on the chosen attack. The parameters
used for the results were Ecase=1 and EI=0.4.
3. Run the "B20_AMD_main.m" script.
4. From the "B20_ATMD_main.m" file, change the parameters needed. The way the parameters are changed is the exact same as 
on step one. Note that the values of FDIon, Ecase and EI should be the same, so that the results from both simulations are
comparable.
5. Run the "B20_ATMD_main.m" script WITHOUT clearing the workspace.
6. Run the "DoS_plots.m" file to generate the relevant plots for this analysis.

FDI attacks:
1. Clear the workspace by typing "clear all" in the command Window.
2. From the "B20_AMD_main.m" file, change the parameters needed. The FDIon variable should be set to 1. The Ecase and EI variables
are irrelevant for this simulation. FDI_amp and FDI_fr can be changed accordingly. The designed attack had FDI_amp=2e6 N and 
FDI_fr=0.278681 Hz
3. Run the "B20_AMD_main.m" script.
4. From the "B20_ATMD_main.m" file, change the parameters needed. The way the parameters are changed is the exact same as 
on step one. The designed attack for the ATMD system had FDI_amp=1e8 N and FDI_fr=0.284687 Hz
5. Run the "B20_ATMD_main.m" script WITHOUT clearing the workspace.
6. Run the "FDI_plots.m" file to generate the relevant plots for this analysis.