package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.sim.SimMechanism.MechanismState;
import frc.robot.sim.SimMotorController;
import frc.robot.sim.SimMotorController.ControllerOutput.CurrentOutput;
import frc.robot.sim.SimMotorController.ControllerOutput.VoltageOutput;
import frc.robot.sim.simController.UnitSafeControl.TrapezoidProfile.State;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.extras.util.mathutils.MeasureMath;
import java.util.Optional;

public class UnitSafeMotorController implements SimMotorController {
  private static final VelocityUnit<AngleUnit> VU = VelocityUnit.combine(Radians, Seconds);
  private static final VelocityUnit<AngularVelocityUnit> AU =
      VelocityUnit.combine(RadiansPerSecond, Seconds);

  private sealed interface Output {
    public ControllerOutput run(Time dt, Voltage supply, MechanismState state);

    public record ClosedLoopOutput<U extends Unit>(
        ClosedLoop<?, U, AngleUnit> controller, Measure<U> value, Velocity<U> secondOrderValue)
        implements Output {
      @Override
      @SuppressWarnings("unchecked")
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        Measure<?> output;
        if (controller().isVelocity()) {
          output =
              controller()
                  .runVelocity(
                      state.position(),
                      State.of(state.velocity(), state.acceleration()),
                      new State<>(
                          (Measure<AngularVelocityUnit>) value,
                          (Velocity<AngularVelocityUnit>) secondOrderValue),
                      dt);
        } else {
          output =
              controller()
                  .runPosition(
                      state.position(),
                      State.of(state.position(), state.velocity()),
                      new State<>(
                          (Measure<AngleUnit>) value, (Velocity<AngleUnit>) secondOrderValue),
                      dt);
        }
        if (output.unit() instanceof VoltageUnit) {
          return ControllerOutput.of((Voltage) output);
        } else {
          return ControllerOutput.of((Current) output);
        }
      }
    }

    public record OpenLoopVoltageOutput(Voltage volts) implements Output {
      @Override
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        return ControllerOutput.of(MeasureMath.clamp(volts, supply));
      }
    }

    public record OpenLoopCurrentOutput(Current amps) implements Output {
      @Override
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        return ControllerOutput.of(amps);
      }
    }

    public static OpenLoopVoltageOutput of(Voltage volts) {
      return new OpenLoopVoltageOutput(volts);
    }

    public static OpenLoopCurrentOutput of(Current amps) {
      return new OpenLoopCurrentOutput(amps);
    }
  }

  public record CurrentLimits(
      Current statorCurrentLimit,
      Current supplyCurrentLimit,
      Current supplyCurrentLowerLimit,
      Time lowerLimitTriggerTime)
      implements StructSerializable {
    public static CurrentLimits base() {
      return new CurrentLimits(Amps.of(120.0), Amps.of(70.0), Amps.of(40.0), Seconds.of(1.0));
    }

    public CurrentLimits times(double factor) {
      return new CurrentLimits(
          statorCurrentLimit.times(factor),
          supplyCurrentLimit.times(factor),
          supplyCurrentLowerLimit.times(factor),
          lowerLimitTriggerTime);
    }

    public static final Struct<CurrentLimits> struct =
        ProceduralStructGenerator.genRecord(CurrentLimits.class);
  }

  public enum SoftLimitTrigger {
    NONE,
    REVERSE,
    FORWARD;

    public static final Struct<SoftLimitTrigger> struct =
        ProceduralStructGenerator.genEnum(SoftLimitTrigger.class);
  }

  private DCMotorExt motor = null;
  private int numMotors = 0;
  private CurrentLimits currentLimits = CurrentLimits.base();
  private Time timeOverSupplyLimit = Seconds.of(0.0);
  private double sensorToMechanismRatio = 1.0;

  private Angle forwardSoftLimit = Radians.of(Double.POSITIVE_INFINITY);
  private Angle reverseSoftLimit = Radians.of(Double.NEGATIVE_INFINITY);

  private Optional<Output> output = Optional.empty();
  private boolean brakeMode = false;

  private Current lastCurrent = Amps.of(0.0);
  private Voltage lastVoltage = Volts.of(0.0);
  private MechanismState lastState = MechanismState.zero();

  public UnitSafeMotorController() {}

  public UnitSafeMotorController configureCurrentLimit(CurrentLimits currentLimit) {
    this.currentLimits = currentLimit;
    return this;
  }

  public UnitSafeMotorController configureSoftLimits(Angle forwardLimit, Angle reverseLimit) {
    this.forwardSoftLimit = forwardLimit;
    this.reverseSoftLimit = reverseLimit;
    return this;
  }

  public UnitSafeMotorController configSensorToMechanismRatio(double ratio) {
    this.sensorToMechanismRatio = ratio;
    return this;
  }

  @Override
  public void configureMotorModel(DCMotorExt motor) {
    this.motor = motor;
    this.numMotors = motor.numMotors;
  }

  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }

    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlCurrent(controller, position, RadiansPerSecond.zero());
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }

    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlCurrent(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  public void controlCurrent(Current amps) {

    output = Optional.of(Output.of(amps));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }

    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlVoltage(controller, position, RadiansPerSecond.zero());
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }

    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlVoltage(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  public void controlVoltage(Voltage volts) {

    output = Optional.of(Output.of(volts));
  }

  public Angle position() {
    return lastState.position();
  }

  public AngularVelocity velocity() {
    return lastState.velocity();
  }

  public AngularAcceleration acceleration() {
    return lastState.acceleration();
  }

  public Current current() {
    return lastCurrent;
  }

  public Voltage voltage() {
    return lastVoltage;
  }

  @Override
  public boolean brakeEnabled() {
    return brakeMode;
  }

  @Override
  public ControllerOutput run(Time dt, Voltage supply, MechanismState rawState) {
    MechanismState state = rawState.div(sensorToMechanismRatio);
    lastState = state;
    return output
        .map(o -> o.run(dt, supply, state))
        .map(o -> softLimit(o, state))
        .map(o -> currentLimit(dt, o, state, supply))
        .orElseGet(() -> ControllerOutput.zero());
  }

  private ControllerOutput softLimit(ControllerOutput requestedOutput, MechanismState state) {
    double direction = requestedOutput.signumMagnitude();
    Angle position = state.position();
    if (direction > 0 && position.in(Radians) > forwardSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    } else if (direction < 0 && position.in(Radians) < reverseSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    }
    return requestedOutput;
  }

  private ControllerOutput currentLimit(
      Time dt, ControllerOutput requestedOutput, MechanismState state, Voltage supplyVoltage) {
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
    final CurrentLimits limits = currentLimits.times(numMotors);
    final AngularVelocity velocity = state.velocity();
    Voltage voltageInput;
    Current statorCurrent;
    Current supplyCurrent;

    if (requestedOutput instanceof VoltageOutput vo) {
      voltageInput = vo.voltage();
      statorCurrent = motor.getCurrent(velocity, voltageInput);
    } else {
      CurrentOutput co = (CurrentOutput) requestedOutput;
      statorCurrent = co.current();
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    }

    voltageInput = MeasureMath.clamp(voltageInput, supplyVoltage);
    supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);

    boolean overStatorLimit = statorCurrent.gt(limits.statorCurrentLimit);
    boolean overSupplyLimit = supplyCurrent.gt(limits.supplyCurrentLimit);
    boolean overLowerSupplyLimit =
        supplyCurrent.gt(limits.supplyCurrentLowerLimit)
            && timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime);

    if (overStatorLimit) {
      statorCurrent = limits.statorCurrentLimit;
      voltageInput = motor.getVoltage(statorCurrent, velocity);
      supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);
    }

    if (overSupplyLimit) {
      timeOverSupplyLimit = timeOverSupplyLimit.plus(dt);
      Current supplyLimit = limits.supplyCurrentLimit;
      if (timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime)) {
        supplyLimit = limits.supplyCurrentLowerLimit;
      }
      statorCurrent =
          motor.getCurrent(velocity, voltageInput, supplyLimit, limits.statorCurrentLimit);
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    } else {
      timeOverSupplyLimit = Seconds.of(0.0);
    }

    lastVoltage = voltageInput;
    lastCurrent = statorCurrent;

    if (requestedOutput instanceof VoltageOutput) {
      return ControllerOutput.of(voltageInput);
    } else {
      return ControllerOutput.of(statorCurrent);
    }
  }
}
