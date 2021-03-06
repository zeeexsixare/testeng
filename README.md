This repository is a basis for the test engineering challenge via Willem Melching.  Challenge text below:
------
This is the comma.ai Test Engineer challenge. The purpose of this challenge is to test
the longitudinal (gas/brake) planning abilities of Openpilot. The longitudinal plan is
done using Model Predictive Control (MPC). For this challenge the planning algorithm can be
treated as a black box. Your goal is to run the planning algorithm through a some scenarios, and show an auto-generated report.

The challenge:
 - Clone the openpilot repository (https://github.com/commaai/openpilot)
 - Rebuild the MPC binaries for x86/x64 instead of arm (cd selfdrive/controls/lib/longitudinal_mpc && make clean && make -j4)
 - Use the provided python file (lateral_mpc.py, long?) as wrapper for MPC. Look at the main function for an example on how to use
 - Build a simple simulor that uses the plan provided by MPC. See lateral_mpc.py (long?) for an example.
 - Run the simulator through the following scenarios:
   - Approaching a stopped car at different speeds (5, 10, 20, 30 m/s from 200m of initial distance)
   - Emergency stop, lead car brakes at 9.81 m/s^2 starting at 30 m/s, from an initial distance of 100m
 - For each maneuver, auto-generate a report that includes:
   - a pass/fail mark, using the following criteria (fail if any of the following is true):
     - The car hits the lead car
     - The optimization algorithm returns an infeasable solution (NaNs in trajectory)
     - The code crashes
  - A plot of position, speed and acceleration of own and lead vehicle (single plot with 3 subplots)

Requirements:
 - Use Pythons built-in unittest framework
 - The tests should run completely inside docker
 - All the plots should be summarized in one report (e.g. one .html or .pdf)
 - Run the tests in a CI environment (e.g. CircleCi), whenever a push is done to your openpilot fork

Bonus points:
 - Fast runtime, for example run tests in parallel.
