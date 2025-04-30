package frc.robot.sim.simMechanism.simBattery;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.simMechanism.SimMechanism;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

/**
 * A very simple battery sim using WPILib’s default 12 V, 0.02 Ω model, but with
 * add/remove‐mechanism support to gather current draws.
 */
public class SimpleBatterySim implements BatterySimInterface {
  private double lastVoltage = 12.0;

  // === Mechanism registry ===
  private final CopyOnWriteArrayList<Supplier<Current>> suppliers = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechMap =
      new ConcurrentHashMap<>();

  @Override
  public void addMechanism(SimMechanism mech) {
    // Wrap the mech’s current draw method in a Supplier<Current>
    Supplier<Current> sup = () -> mech.motorVariables().statorCurrent();
    mechMap.put(mech, sup);
    suppliers.add(sup);
  }

  @Override
  public boolean removeMechanism(SimMechanism mech) {
    Supplier<Current> sup = mechMap.remove(mech);
    return sup != null && suppliers.remove(sup);
  }

  @Override
  public void update(double dt) {
    // 1) Sum all currents (I > 0 = discharge)
    double totalAmps =
        suppliers.stream().mapToDouble(s -> s.get().in(edu.wpi.first.units.Units.Amps)).sum();

    // 2) Use WPILib’s single‐resistor sag model
    double loaded = BatterySim.calculateDefaultBatteryLoadedVoltage(totalAmps);

    // 3) Clamp to [0,12]
    lastVoltage = MathUtil.clamp(loaded, 0, 12);

    // 4) Push into RoboRioSim
    RoboRioSim.setVInVoltage(lastVoltage);
    RoboRioSim.setVInCurrent(totalAmps);
  }

  @Override
  public Voltage getVoltage() {
    return Volts.of(lastVoltage);
  }

  @Override
  public void reset() {
    lastVoltage = 12.0;
    RoboRioSim.setVInVoltage(lastVoltage);
    RoboRioSim.setVInCurrent(0.0);
  }
}
