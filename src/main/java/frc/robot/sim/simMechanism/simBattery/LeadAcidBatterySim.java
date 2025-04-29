package frc.robot.sim.simMechanism.simBattery;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.simMechanism.SimMechanism;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

/**
 * A lead–acid battery simulation using a two-RC-branch Thévenin model. The simulation tracks the
 * battery's state of charge (SOC), open circuit voltage (OCV), and polarization effects over time
 * using a simple electrical model with resistors and capacitors.
 */
public class LeadAcidBatterySim {
  // === Battery Parameters ===
  private final double QNom, R0, Rp1, Cp1, Rp2, Cp2;

  // === Battery State ===
  private double soc = 1.0; // State of Charge (SOC), 1.0 is fully charged
  private double vp1 = 0.0, vp2 = 0.0; // Polarization voltages across the two RC branches

  // === Last Voltage ===
  private double lastVoltage = 12.0; // Initially, the battery voltage is nominal (12V)

  // === Appliances and Mechanisms ===
  private final CopyOnWriteArrayList<Supplier<Current>> appliances = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechSuppliers =
      new ConcurrentHashMap<>();

  /**
   * Constructor to initialize the battery parameters.
   *
   * @param capacityAh Nominal capacity of the battery in amp-hours (Ah)
   * @param r0 Instantaneous resistance (Ω)
   * @param rp1, cp1 Parameters for the first RC branch (resistance in Ω and capacitance in F)
   * @param rp2, cp2 Parameters for the second RC branch (resistance in Ω and capacitance in F)
   */
  public LeadAcidBatterySim(
      double capacityAh, double r0, double rp1, double cp1, double rp2, double cp2) {
    this.QNom = capacityAh * 3600.0; // Convert Ah to Coulombs (1 Ah = 3600 C)
    this.R0 = r0;
    this.Rp1 = rp1;
    this.Cp1 = cp1;
    this.Rp2 = rp2;
    this.Cp2 = cp2;
  }

  /**
   * Register a new SimMechanism so its current draw contributes to the battery load. The
   * mechanism's current draw is added to the total current of the battery.
   */
  public void addMechanism(SimMechanism mech) {
    Supplier<Current> supplier = () -> mech.motorVariables().statorCurrent();
    mechSuppliers.put(mech, supplier);
    appliances.add(supplier);
  }

  /** Unregister a SimMechanism so its current draw is removed from the battery load. */
  public boolean removeMechanism(SimMechanism mech) {
    Supplier<Current> supplier = mechSuppliers.remove(mech);
    if (supplier != null) {
      return appliances.remove(supplier);
    }
    return false;
  }

  /**
   * Advances the simulation by a time step of dt seconds, updates RoboRioSim, and returns the new
   * voltage.
   *
   * @param dt Time step in seconds (e.g., 0.02 for 20 ms)
   */
  public void update(double dt) {
    // === Step 1: Sum all currents to get the total current draw ===
    // Appliances are the devices that draw current from the battery (e.g., motors).
    // We sum the current from each appliance to calculate the total current.
    double totalAmps = appliances.stream().mapToDouble(s -> s.get().in(Units.Amps)).sum();

    // === Step 2: Update State of Charge (SOC) using Coulomb counting ===
    // SOC decreases as the battery is discharged (totalAmps * dt) gives the amount of charge used.
    // We divide by QNom (total charge capacity) to get the new SOC.
    soc = Math.max(0.0, Math.min(1.0, soc - (totalAmps * dt) / QNom)); // Clamp SOC to [0, 1]

    // === Step 3: Update RC Branch Polarization Dynamics ===
    // Polarization effects are modeled using two RC circuits (short-term and long-term).
    // The voltage across each branch (vp1 and vp2) is updated based on the current draw and RC time
    // constants.
    double tau1 = Rp1 * Cp1; // Time constant for the first RC branch
    double tau2 = Rp2 * Cp2; // Time constant for the second RC branch
    vp1 += ((-vp1 / tau1) + (totalAmps / Cp1)) * dt; // Update voltage for short-term polarization
    vp2 += ((-vp2 / tau2) + (totalAmps / Cp2)) * dt; // Update voltage for long-term polarization

    // === Step 4: Calculate Open Circuit Voltage (OCV) based on SOC ===
    // The OCV depends on the SOC. For this simulation, we use a simple linear approximation.
    double ocv = 11.8 + 0.9 * soc; // OCV is between 11.8V at 0% SOC and 12.7V at 100% SOC

    // === Step 5: Calculate the Terminal Voltage ===
    // The terminal voltage is the OCV minus the voltage drops from the internal resistance and
    // polarization effects.
    lastVoltage = ocv - totalAmps * R0 - vp1 - vp2;

    // Ensure that the voltage stays within the reasonable range of 0V to 12V
    lastVoltage = Math.max(0.0, Math.min(12.0, lastVoltage));

    // === Step 6: Update the RoboRioSim framework with the new voltage ===
    // This allows the simulated battery voltage to be used in the robot's control system.
    RoboRioSim.setVInVoltage(lastVoltage);
  }

  /**
   * @return The most recently simulated battery voltage.
   */
  public Voltage getLastVoltage() {
    return Units.Volts.of(lastVoltage);
  }
}
