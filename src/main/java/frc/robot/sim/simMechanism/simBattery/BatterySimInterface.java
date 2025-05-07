package frc.robot.sim.simMechanism.simBattery;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.simMechanism.SimMechanism;

/** Common interface for all battery simulation models. */
public interface BatterySimInterface {
  /**
   * Advance the battery simulation by dt seconds.
   *
   * @param dt Time step in seconds (e.g. 0.02 for 20 ms)
   */
  void update(double I, double dt);

  /**
   * @return The latest terminal voltage [V]
   */
  Voltage getVoltage();

  /** Reset all internal state back to fully-charged, nominal conditions. */
  void reset();

  /**
   * Add a mechanism to the battery simulation.
   *
   * @param mech The mechanism to add. The battery sim will track its current draw.
   */
  void addMechanism(SimMechanism mech);

  /**
   * Remove a mechanism from the battery simulation.
   *
   * @param mech The mechanism to remove. The battery sim will stop tracking its current draw.
   * @return true if the mechanism was successfully removed, false otherwise.
   */
  boolean removeMechanism(SimMechanism mech);
}
