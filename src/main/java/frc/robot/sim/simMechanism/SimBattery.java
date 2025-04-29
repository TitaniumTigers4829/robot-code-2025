package frc.robot.sim.simMechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

public class SimBattery {
  private final CopyOnWriteArrayList<Supplier<Current>> appliances = new CopyOnWriteArrayList<>();
  private final ConcurrentHashMap<SimMechanism, Supplier<Current>> mechSuppliers =
      new ConcurrentHashMap<>();

  private Voltage lastVoltage = Volts.of(12);

  public void addMechanism(SimMechanism simMechanism) {
    Supplier<Current> supplier = () -> simMechanism.getMotorCurrent(lastVoltage);
    mechSuppliers.put(simMechanism, supplier);
    appliances.add(supplier);
  }

  public boolean removeMechanism(SimMechanism simMechanism) {
    Supplier<Current> supplier = mechSuppliers.remove(simMechanism);
    if (supplier != null) {
      return appliances.remove(supplier);
    }
    return false;
  }

  public Voltage updateAndGetBatteryVoltage() {
    double totalAmps = appliances.stream().mapToDouble(s -> s.get().in(Amps)).sum();
    double loaded = BatterySim.calculateDefaultBatteryLoadedVoltage(totalAmps);
    // BatterySim.calculateLoadedBatteryVoltage(totalAmps, loaded, null)
    lastVoltage = Volts.of(MathUtil.clamp(loaded, 0, 12));
    RoboRioSim.setVInVoltage(lastVoltage.in(Volts));
    return lastVoltage;
  }

  public Voltage getLastVoltage() {
    return lastVoltage;
  }
}
