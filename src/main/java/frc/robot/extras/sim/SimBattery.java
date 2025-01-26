package frc.robot.extras.sim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

public class SimBattery {
  private final ConcurrentHashMap<Object, Supplier<Current>> electricalAppliances =
      new ConcurrentHashMap<>();

  public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
    this.electricalAppliances.put(new Object(), customElectricalAppliances);
  }

  public void addMechanism(SimMechanism simMechanism) {
    this.electricalAppliances.put(simMechanism, () -> simMechanism.inputs().supplyCurrent());
  }

  public void removeMechanism(SimMechanism simMechanism) {
    this.electricalAppliances.remove(simMechanism);
  }

  public Voltage getBatteryVoltage() {
    final double[] totalCurrentAmps =
        electricalAppliances.values().stream()
            .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
            .toArray();

    double batteryVoltageVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);

    return Volts.of(MathUtil.clamp(batteryVoltageVolts, 0, 12));
  }
}
