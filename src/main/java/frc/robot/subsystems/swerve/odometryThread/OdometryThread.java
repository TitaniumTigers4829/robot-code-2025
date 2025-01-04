package frc.robot.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Robot;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface OdometryThread {
  final class OdometryDoubleInput {
    private final Supplier<Angle> supplier;
    private final Queue<Angle> queue;

    public OdometryDoubleInput(Supplier<Angle> signal) {
      this.supplier = signal;
      // Create a queue with a capacity of 10
      this.queue = new ArrayBlockingQueue<>(10);
    }

    public void cacheInputToQueue() {
      this.queue.offer(supplier.get());
    }
  }

  List<OdometryDoubleInput> registeredInputs = new ArrayList<>();
  List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();

  static Queue<Angle> registerSignalInput(StatusSignal<Angle> signal) {
    signal.setUpdateFrequency(HardwareConstants.SIGNAL_FREQUENCY, HardwareConstants.TIMEOUT_S);
    registeredStatusSignals.add(signal);
    return registerInput(signal.asSupplier());
  }

  static Queue<Angle> registerInput(Supplier<Angle> supplier) {
    final OdometryDoubleInput odometryDoubleInput = new OdometryDoubleInput(supplier);
    registeredInputs.add(odometryDoubleInput);
    return odometryDoubleInput.queue;
  }

  static OdometryThread createInstance(DeviceCANBus canBus) {
    return switch (Constants.CURRENT_MODE) {
      case REAL ->
          new OdometryThreadReal(
              canBus,
              registeredInputs.toArray(new OdometryDoubleInput[0]),
              registeredStatusSignals.toArray(new BaseStatusSignal[0]));
      case SIM -> new OdometryThreadSim();
        // case REPLAY -> inputs -> {};
      default -> throw new IllegalArgumentException("Unexpected value: " + Constants.CURRENT_MODE);
    };
  }

  @AutoLog
  public class OdometryThreadInputs {
    public double[] measurementTimeStamps = new double[0];
  }

  default void updateInputs(OdometryThreadInputs inputs) {}

  default void start() {}

  default void lockOdometry() {}

  default void unlockOdometry() {}

  final class OdometryThreadSim implements OdometryThread {
    @Override
    public void updateInputs(OdometryThreadInputs inputs) {
      inputs.measurementTimeStamps = new double[SimulationConstants.SIMULATION_TICKS_IN_1_PERIOD];
      final double robotStartingTimeStamps = TimeUtil.getLogTimeSeconds(),
          iterationPeriodSeconds =
              Robot.defaultPeriodSecs / SimulationConstants.SIMULATION_TICKS_IN_1_PERIOD;
      for (int i = 0; i < SimulationConstants.SIMULATION_TICKS_IN_1_PERIOD; i++)
        inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
    }
  }
}
