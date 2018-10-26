#!/usr/bin/env python2
import math
import numpy as np
import matplotlib #added
matplotlib.use('svg') #added

from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU


class SimpleLongitudinalMpc(object):
  def __init__(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(1)

    # Alocate some memory for the C++ code to read/store the solution
    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.reset()

  def reset(self):
    """Reset the current state, and initialize the cost functions"""
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

  def init(self, v_ego, x_lead, v_lead, a_lead):
    """Caclulate an initial solution for the optmization algorithm.
    MPC uses a hot-started optimization algorithm, which means it starts
    optimizing using the previous solution. When the scenario changes significantly
    an initial solution needs to be calculated that satisfies all the constraints."""
    a_lead_tau = _LEAD_ACCEL_TAU
    a_lead_tau = max(a_lead_tau, (a_lead**2 * math.pi) / (2 * (v_lead + 0.01)**2))
    self.libmpc.init_with_simulation(v_ego, x_lead, v_lead, a_lead, a_lead_tau)

  def get_trajectory(self, cur_t=0., cur_x=0.):
    """Returns the current estimated trajectory (note the timesteps are not uniform!).
    Since MPC runs only one iteration of the optimization, this trajectory
    may need to converge during the first few iterations."""
    x_ego = np.asarray(list(self.mpc_solution[0].x_ego)) + cur_x
    v_ego = np.asarray(list(self.mpc_solution[0].v_ego))
    a_ego = np.asarray(list(self.mpc_solution[0].a_ego))
    x_lead = np.asarray(list(self.mpc_solution[0].x_l))
    v_lead = np.asarray(list(self.mpc_solution[0].v_l))

    ts = np.hstack([np.arange(0., 1.0, 0.2), np.arange(1.0, 10.1, 0.6)]) + cur_t
    return ts, x_ego, v_ego, a_ego, x_lead, v_lead

  def is_trajectory_valid(self):
    """When the optimization algorithm fails, for example if the
    contstraints are not satisfied, there will be NaNs in the solution.
    This function chekcs the validity of the planned trajectory"""

    ts, x_ego, v_ego, a_ego, x_lead, v_lead = self.get_trajectory()

    no_nans = not np.any([
      np.isnan(x_ego),
      np.isnan(v_ego),
      np.isnan(a_ego),
      np.isnan(x_lead),
      np.isnan(v_lead)])
    # TODO: Add more checks here

    return no_nans

  def run(self, v_ego, a_ego, x_lead, v_lead, a_lead):
    """Run MPC for one timestep. MPC is a moving horizon control scheme,
    the complete trajectory for the next 10 s is optimized, but only one
    timestep is executed (0.2 s). After one timestep the algorithm runs again.

    v_ego: Speed of own vehicle (m/s)
    a_ego: Acceleration of own vehicle (m/s^2)
    x_lead: Distance of lead vehicle (m)
    v_lead: Speed of lead vehicle (m/s)
    a_lead: Acceleration of lead vehicle (m/s^2)

    Timesteps are non-uniform, the first 5 steps are 0.2s, the next 15 are 0.6 s.
    Returns action to take (speed, acceleration) during the next timestep"""

    # This is a time constant for how long the lead car will maintain the current acceleration
    # It is considered static for this simple version of MPC
    a_lead_tau = _LEAD_ACCEL_TAU
    a_lead_tau = max(a_lead_tau, (a_lead**2 * math.pi) / (2 * (v_lead + 0.01)**2))

    self.cur_state[0].v_ego = v_ego
    self.cur_state[0].a_ego = a_ego
    self.cur_state[0].x_l = x_lead
    self.cur_state[0].v_l = v_lead

    self.libmpc.run_mpc(self.cur_state, self.mpc_solution, a_lead_tau, a_lead)

    # Read solution
    v_mpc = self.mpc_solution[0].v_ego[1]
    a_mpc = self.mpc_solution[0].a_ego[1]
    return (v_mpc, a_mpc)


if __name__ == "__main__":
  import matplotlib.pyplot as plt
  dt = 0.2
  mpc = SimpleLongitudinalMpc()

  # Initial conditions
  t = 0.
  x_ego = 0.
  v_ego = 20.
  a_ego = 0

  x_lead = 200.
  v_lead = 0.
  a_lead = 0.

  # Get initial plan
  mpc.init(v_ego, x_lead, v_lead, a_lead)
  mpc.run(v_ego, a_ego, x_lead - x_ego, v_lead, a_lead)

  # Run small simulation
  ts = []
  v_egos = []
  x_egos = []

  for k in range(150):
    v_ego, a_ego = mpc.run(v_ego, a_ego, x_lead - x_ego, v_lead, a_lead)
    assert mpc.is_trajectory_valid()
    x_ego += v_ego * dt

    ts.append(t)
    x_egos.append(x_ego)
    v_egos.append(v_ego)

    t += dt

  # Plot results
  plt.subplot(2, 1, 1)
  plt.plot(ts, x_egos, label='x_ego')
  plt.plot(ts, len(ts) * [x_lead], 'k--', label='x_lead')
  plt.xlabel('time [s]')
  plt.ylabel('x [m]')
  plt.legend()

  plt.subplot(2, 1, 2)
  plt.plot(ts, v_egos, label='v_ego')
  plt.plot(ts, len(ts) * [v_lead], 'k--', label='v_lead')
  plt.xlabel('time [s]')
  plt.ylabel('v [m/s]')
  plt.legend()

  print "Stopping distance: %.2f m" % (x_lead - x_ego)

  plt.show()
