package frc.robot.sim.simMechanism.simBattery;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.simMechanism.SimMechanism;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

/** A lead–acid battery simulation using a two-RC-branch Thévenin model */
public class LeadAcidBatterySim {
  // Nominal capacity [C], ohmic resistance [Ω], and two RC branches
  private final double QNom, R0, Rp1, Cp1, Rp2, Cp2;

  // Dynamic states
  private double soc = 1.0;
  private double vp1 = 0.0, vp2 = 0.0;

  // Last computed terminal voltage [V]
  private double lastVoltage = 12.0;

  // Suppliers for current draw
  private final CopyOnWriteArrayList<Supplier<Current>> appliances = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechSuppliers =
      new ConcurrentHashMap<>();

  /**
   * @param capacityAh Nominal capacity in amp-hours
   * @param r0 Instantaneous resistance (Ω)
   * @param rp1, cp1 First RC branch (Ω, F)
   * @param rp2, cp2 Second RC branch (Ω, F)
   */
  public LeadAcidBatterySim(
      double capacityAh,
      double r0,
      double rp1, double cp1,
      double rp2, double cp2) {
    this.QNom = capacityAh * 3600.0;  // Ah → Coulombs
    this.R0 = r0;
    this.Rp1 = rp1;  this.Cp1 = cp1;
    this.Rp2 = rp2;  this.Cp2 = cp2;
  }

  /**
   * Register a new SimMechanism so its current draw contributes to
   * the battery load.
   */
  public void addMechanism(SimMechanism mech) {
    Supplier<Current> supplier = () -> mech.motorVariables().statorCurrent();
    mechSuppliers.put(mech, supplier);
    appliances.add(supplier);
  }

  /**
   * Unregister a SimMechanism so its draw is removed from the load.
   */
  public boolean removeMechanism(SimMechanism mech) {
    Supplier<Current> supplier = mechSuppliers.remove(mech);
    if (supplier != null) {
      return appliances.remove(supplier);
    }
    return false;
  }

  /**
   * Advance the simulation by dt seconds, update RoboRioSim, and return the new voltage.
   *
   * @param dt Time step [s] (e.g. 0.02 for 20 ms)
   */
  public void update(double dt) {
    // 1) Sum all currents (I > 0 = discharge)
    double totalAmps = appliances.stream()
        .mapToDouble(s -> s.get().in(Units.Amps))
        .sum();

    // 2) Coulomb counting → SOC (clamp 0–1)
    soc = Math.max(0.0, Math.min(1.0, soc - (totalAmps * dt) / QNom));

    // 3) RC-branch polarization dynamics
    double tau1 = Rp1 * Cp1;
    double tau2 = Rp2 * Cp2;
    vp1 += ((-vp1 / tau1) + (totalAmps / Cp1)) * dt;
    vp2 += ((-vp2 / tau2) + (totalAmps / Cp2)) * dt;

    // 4) OCV from SOC (simple linear approx: 11.8 V @0% → 12.7 V @100%)
    double ocv = 11.8 + 0.9 * soc;

    // 5) Terminal voltage = OCV – IR0 – Vp1 – Vp2
    lastVoltage = ocv - totalAmps * R0 - vp1 - vp2;
    lastVoltage = Math.max(0.0, Math.min(12.0, lastVoltage));

    // 6) Push into WPILib sim framework
    RoboRioSim.setVInVoltage(lastVoltage);

    // return Units.Volts.of(lastVoltage);
  }

  /** @return The most recently simulated battery voltage. */
  public Voltage getLastVoltage() {
    return Units.Volts.of(lastVoltage);
  }
}
