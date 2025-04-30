package frc.robot.sim.simMechanism.simBattery;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.simMechanism.SimMechanism;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

/**
 * A lead–acid battery simulation using a two-RC-branch Thévenin model. Tracks State of Charge
 * (SOC), Open-Circuit Voltage (OCV), and voltage sag.
 */
public class LeadAcidBatterySim {
  // === Battery Parameters ===
  private final double QNom; // total charge capacity in Coulombs
  private final double R0; // instantaneous (ohmic) resistance [Ω]
  private final double Rp1; // polarization resistance 1 [Ω]
  private final double Cp1; // polarization capacitance 1 [F]
  private final double Rp2; // polarization resistance 2 [Ω]
  private final double Cp2; // polarization capacitance 2 [F]

  // === Dynamic State ===
  private double soc = 1.0; // State-of-Charge (0.0–1.0)
  private double vp1 = 0.0; // voltage across RC branch 1 (short-term)
  private double vp2 = 0.0; // voltage across RC branch 2 (long-term)
  private double lastVoltage = 12.0; // last computed terminal voltage [V]

  // List of current-drawing suppliers from each mechanism
  private final CopyOnWriteArrayList<Supplier<Current>> appliances = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechSuppliers =
      new ConcurrentHashMap<>();

  /**
   * @param capacityAh Battery capacity in amp-hours
   * @param r0 Ohmic resistance [Ω]
   * @param rp1, cp1 RC branch 1 parameters [Ω], [F]
   * @param rp2, cp2 RC branch 2 parameters [Ω], [F]
   */
  public LeadAcidBatterySim(
      double capacityAh, double r0, double rp1, double cp1, double rp2, double cp2) {
    this.QNom = capacityAh * 3600.0; // convert Ah → C
    this.R0 = r0;
    this.Rp1 = rp1;
    this.Cp1 = cp1;
    this.Rp2 = rp2;
    this.Cp2 = cp2;
  }

  /** Adds a mechanism’s current draw into the battery load. */
  public void addMechanism(SimMechanism mech) {
    Supplier<Current> sup = () -> mech.motorVariables().statorCurrent();
    mechSuppliers.put(mech, sup);
    appliances.add(sup);
  }

  /** Removes a mechanism from contributing to the load. */
  public boolean removeMechanism(SimMechanism mech) {
    Supplier<Current> sup = mechSuppliers.remove(mech);
    return sup != null && appliances.remove(sup);
  }

  /**
   * Step the battery sim by dt seconds. Calculates SOC, OCV, polarization, sag, and updates
   * RoboRioSim.
   */
  public void update(double dt) {
    // 1) Sum all current draws (I > 0 = discharge)
    double I = appliances.stream().mapToDouble(s -> s.get().in(Units.Amps)).sum();

    // 2) Update State-of-Charge via Coulomb counting
    soc = Math.max(0.0, Math.min(1.0, soc - (I * dt) / QNom));

    // 3) Update polarization voltages for each RC branch
    double tau1 = Rp1 * Cp1;
    double tau2 = Rp2 * Cp2;
    vp1 += ((-vp1 / tau1) + (I / Cp1)) * dt;
    vp2 += ((-vp2 / tau2) + (I / Cp2)) * dt;

    // 4) Compute Open-Circuit Voltage from SOC (linear approx)
    double ocv = 11.8 + 0.9 * soc; // 11.8 V at 0% → 12.7 V at 100%

    // 5) Calculate each component of voltage sag
    double irDrop = I * R0; // Ohmic drop
    double polDrop = vp1 + vp2; // Total polarization drop
    double sag = irDrop + polDrop; // Combined sag

    // 6) Terminal voltage = OCV − sag
    lastVoltage = ocv - sag;
    // Clamp between 0 and OCV to avoid negative or >OCV readings
    lastVoltage = Math.max(0.0, Math.min(ocv, lastVoltage));

    // 7) Push into WPILib sim (robot code reads this as battery voltage)
    RoboRioSim.setVInVoltage(lastVoltage);

    // maybe log amps drawn by each mechanism?
    for (var entry : mechSuppliers.entrySet()) {
        SimMechanism mech = entry.getKey();
        double amps = entry.getValue().get().in(Units.Amps);
        // System.out.println(mech. + " drawing " + amps + " A");
      }

//       mechSuppliers.put(badMech, () -> Current.ofAmps(50.0));  
// // ensure appliances list is in sync
// appliances.remove(oldSupplier);
// appliances.add(mechSuppliers.get(badMech));

// Supplier<Current> realSup = mechSuppliers.get(mech);
// Supplier<Current> zeroSup = () -> Current.ofAmps(0.0);
// mechSuppliers.put(mech, zeroSup);
// appliances.replaceAll(s -> s == realSup ? zeroSup : s);

// for (var entry : mechSuppliers.entrySet()) {
//     SimMechanism mech = entry.getKey();
//     Supplier<Current> sup  = entry.getValue();
//     Current I = sup.get();
//     // e.g. if mech has stalled flag:
//     if (mech.isStalled()) {
//       I = Current.ofAmps(I.in(Units.Amps) * 2.0);
//     }
//     totalAmps += I.in(Units.Amps);
//   }
// mechSuppliers.put(mech, () -> mech.isEnabled()
// ? mech.motorVariables().statorCurrent()
// : Current.ofAmps(0));

      
  }

  /**
   * @return the most recently simulated terminal voltage.
   */
  public Voltage getLastVoltage() {
    return Units.Volts.of(lastVoltage);
  }
}
