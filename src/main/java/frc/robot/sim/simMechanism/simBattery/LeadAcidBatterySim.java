package frc.robot.sim.simMechanism.simBattery;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.simMechanism.SimMechanism;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates a 12 V lead–acid battery using a two-RC-branch Thévenin model.
 *
 * <p>Tracks: • bulkSOC – true State-of-Charge from coulomb counting • surfaceSOC – effective SOC
 * for voltage reading (models electrolyte stratification) • vp1/vp2 – voltages on short- and
 * long-time-constant polarization branches • temperature effects on R₀
 *
 * <p>On each update(dt): 1) Sum currents → I 2) bulkSOC ← bulkSOC − I·dt/Q 3) surfaceSOC recovers
 * toward bulkSOC (faster at rest) 4) vp1, vp2 integrate dvp/dt = −vp/τ + I/C 5) ocv = f(surfaceSOC)
 * 6) sag = I·R₀(T) + vp1 + vp2 7) Vterm = ocv − sag → RoboRioSim
 */
public class LeadAcidBatterySim implements BatterySimInterface {
  // === Constructor Parameters ===
  private final double QNom; // capacity [C]
  private final double R0_ref; // base ohmic resistance [Ω]
  private final double Rp1, Cp1, Rp2, Cp2; // RC branch params
  private final double tau1, tau2; // cached time constants

  // === Recovery ===
  // TODO: Make an actual Simulation constants
  private static final double DEFAULT_RECOVERY_TAU = 500.0; // [s]
  private static final double REST_FACTOR = 5.0; // faster recovery at rest

  // === State Variables ===
  private double bulkSOC = 1.0; // true coulomb-count SOC
  private double surfaceSOC = 1.0; // “observable” SOC for voltage
  private double vp1 = 0.0, vp2 = 0.0; // polarization voltages
  private double lastVoltage = 12.0; // terminal voltage [V]
  private double ambientTemp = 25.0; // [°C]

  // === Mechanism Current Suppliers ===
  private final CopyOnWriteArrayList<Supplier<Current>> appliances = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechMap =
      new ConcurrentHashMap<>();

  /**
   * @param capacityAh the battery’s nominal capacity in amp-hours [Ah], e.g. 18 for an 18 Ah
   *     battery
   * @param r0 the base (ohmic) resistance R₀ at 25 °C, in ohms [Ω]
   * @param rp1 the resistance of the short-term polarization branch, in ohms [Ω]
   * @param cp1 the capacitance of the short-term polarization branch, in farads [F]
   * @param rp2 the resistance of the long-term polarization branch, in ohms [Ω]
   * @param cp2 the capacitance of the long-term polarization branch, in farads [F]
   */
  public LeadAcidBatterySim(
      double capacityAh, double r0, double rp1, double cp1, double rp2, double cp2) {
    this.QNom = capacityAh * 3600.0; // Ah → C
    this.R0_ref = r0;
    this.Rp1 = rp1;
    this.Cp1 = cp1;
    this.Rp2 = rp2;
    this.Cp2 = cp2;
    this.tau1 = rp1 * cp1; // R·C time constants
    this.tau2 = rp2 * cp2;
  }

  /** Register a mechanism’s current draw. */
  @Override
  public void addMechanism(SimMechanism mech) {
    Supplier<Current> sup = () -> mech.motorVariables().supplyCurrent();
    Logger.recordOutput(
        "LeadAcidBattery/supplyCurrent " + mech.name(),
        mech.motorVariables().supplyCurrent().in(Amps));
    Logger.recordOutput(
        "LeadAcidBattery/supplyVoltage " + mech.name(),
        mech.motorVariables().supplyVoltage().in(Volts));
    Logger.recordOutput(
        "LeadAcidBattery/statorVoltage " + mech.name(),
        mech.motorVariables().statorVoltage().in(Volts));
    Logger.recordOutput(
        "LeadAcidBattery/statorCurrent " + mech.name(),
        mech.motorVariables().statorCurrent().in(Amps));
    mechMap.put(mech, sup);
    appliances.add(sup);
  }

  /** Unregister a mechanism’s draw. */
  @Override
  public boolean removeMechanism(SimMechanism mech) {
    Supplier<Current> sup = mechMap.remove(mech);
    return sup != null && appliances.remove(sup);
  }

  /** Set ambient temperature [°C] for R₀ adjustment. */
  public void setTemperature(double tempC) {
    this.ambientTemp = tempC;
  }

  /** Advance simulation by dt seconds. */
  @Override
  public void update(double dt) {
    // 1) Sum all mechanism currents (A)
    double I = appliances.stream().mapToDouble(s -> s.get().in(Units.Amps)).sum();

    // 2) bulkSOC: Coulomb counting dSOC = I·dt / QNom
    bulkSOC = clamp(bulkSOC - (I * dt) / QNom, 0.0, 1.0);
    Logger.recordOutput("LeadAcidBattery/bulkSOC", bulkSOC);

    // 3) surfaceSOC: recovery toward bulkSOC (electrolyte stratification)
    double rate =
        (Math.abs(I) < 1e-2)
            ? 1.0 / (DEFAULT_RECOVERY_TAU / REST_FACTOR)
            : 1.0 / DEFAULT_RECOVERY_TAU;
    surfaceSOC = clamp(surfaceSOC + (bulkSOC - surfaceSOC) * rate * dt, 0.0, 1.0);
    Logger.recordOutput("LeadAcidBattery/surfaceSOC", surfaceSOC);

    // 4) Polarization: dvp/dt = -vp/τ + I/C
    vp1 += ((-vp1 / tau1) + (I / Cp1)) * dt;
    vp2 += ((-vp2 / tau2) + (I / Cp2)) * dt;

    // 5) OCV vs. SOC: logistic-style curve with knee at ~60%
    double ocv = computeOCV(surfaceSOC);
    Logger.recordOutput("LeadAcidBattery/ocv", ocv);

    // 6) Temperature adjust R0: +0.5% per °C below 25
    double tempFactor = ambientTemp < 25.0 ? 1.0 + 0.005 * (25 - ambientTemp) : 1.0;
    double R0 = R0_ref * tempFactor;

    // 7) Voltage Sag components
    double irDrop = I * R0; // instantaneous IR drop
    double polDrop = vp1 + vp2; // RC polarization drops
    double sag = irDrop + polDrop;
    Logger.recordOutput("LeadAcidBattery/sag", sag);
    Logger.recordOutput("LeadAcidBattery/irDrop", irDrop);
    Logger.recordOutput("LeadAcidBattery/polDrop", polDrop);

    // 8) Terminal voltage = OCV − sag, clamped [0, OCV]
    lastVoltage = clamp(ocv - sag, 0.0, ocv);

    // 9) Push into WPILib sim framework
    RoboRioSim.setVInVoltage(lastVoltage);
    // TODO: Check out the following:
    // RoboRioSim.setVInCurrent(I);
    Logger.recordOutput("LeadAcidBattery/terminalVoltage", lastVoltage);
    Logger.recordOutput("LeadAcidBattery/controllerVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("LeadAcidBattery/current", I);
    Logger.recordOutput("LeadAcidBattery/dt", dt);
  }

  /**
   * @return the most recent terminal voltage (V).
   */
  @Override
  public Voltage getVoltage() {
    return Units.Volts.of(lastVoltage);
  }

  /** Reset to fully-charged, zero polarization. */
  @Override
  public void reset() {
    bulkSOC = 1.0;
    surfaceSOC = 1.0;
    vp1 = vp2 = 0.0;
    lastVoltage = computeOCV(surfaceSOC);
    RoboRioSim.setVInVoltage(lastVoltage);
    RoboRioSim.resetData();
  }

  // ─── Helpers ──────────────────────────────────────────────────────────────

  /** Logistic-like OCV curve: ~11.8 V (0%) → 12.7 V (100%), knee at 60%. */
  private double computeOCV(double soc) {
    soc = clamp(soc, 0.01, 0.99);
    return 11.8 + 0.9 * soc;
    // / (1.0 + Math.exp(-4.0 * (soc - 0.6))); // exponent 4 instead of 10
  }

  /** Clamp x into [min, max]. */
  private static double clamp(double x, double min, double max) {
    return x < min ? min : (x > max ? max : x);
  }
}
