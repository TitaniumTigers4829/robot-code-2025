package frc.robot.extras.sim.simController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.extras.util.mathutils.MeasureMath;
import org.littletonrobotics.junction.Logger;

public class UnitSafeControl {
  /** A PD controller that uses units to ensure that the controller is used correctly. */
  public static class PIDFeedback<O extends Unit, Q extends Unit> {
    private final edu.wpi.first.math.controller.PIDController internalController;
    private final O outputUnit;
    private final Q inputUnit;

    public PIDFeedback(Per<O, Q> kP, Per<O, Q> kI, Per<O, VelocityUnit<Q>> kD) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController =
          new edu.wpi.first.math.controller.PIDController(
              kP.baseUnitMagnitude(), kI.baseUnitMagnitude(), kD.baseUnitMagnitude());
    }

    public PIDFeedback(Per<O, Q> kP, Per<O, VelocityUnit<Q>> kD) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController =
          new edu.wpi.first.math.controller.PIDController(
              kP.baseUnitMagnitude(), 0.0, kD.baseUnitMagnitude());
    }

    public PIDFeedback(Per<O, Q> kP) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController =
          new edu.wpi.first.math.controller.PIDController(kP.baseUnitMagnitude(), 0.0, 0.0);
    }

    public void logGains() {
      Logger.recordOutput("Sim/kP", internalController.getP());
      Logger.recordOutput("Sim/kI", internalController.getI());
      Logger.recordOutput("Sim/kD", internalController.getD());
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", inputUnit.name());
    }

    @SuppressWarnings("unchecked")
    public Pair<Measure<O>, Measure<Q>> calculate(Measure<Q> measurement, Measure<Q> setpoint) {
      var o =
          (Measure<O>)
              outputUnit.of(
                  internalController.calculate(
                      measurement.baseUnitMagnitude(), setpoint.baseUnitMagnitude()));
      var error = (Measure<Q>) inputUnit.of(internalController.getError());
      return new Pair<>(o, error);
    }

    public PIDFeedback<O, Q> withTolerance(Measure<Q> tolerance) {
      internalController.setTolerance(tolerance.baseUnitMagnitude());
      return this;
    }

    public PIDFeedback<O, Q> withTolerance(
        Measure<Q> positionTolerance, Measure<Q> velocityTolerance) {
      internalController.setTolerance(
          positionTolerance.baseUnitMagnitude(), velocityTolerance.baseUnitMagnitude());
      return this;
    }

    public PIDFeedback<O, Q> withContinuousInput(Measure<Q> minimumInput, Measure<Q> maximumInput) {
      internalController.enableContinuousInput(
          minimumInput.baseUnitMagnitude(), maximumInput.baseUnitMagnitude());
      return this;
    }

    O getOutputUnit() {
      return outputUnit;
    }

    Q getInputUnit() {
      return inputUnit;
    }
  }

  public static class LinearPIDFeedback<O extends Unit> extends PIDFeedback<O, DistanceUnit> {

    public LinearPIDFeedback(
        Per<O, DistanceUnit> kP,
        Per<O, DistanceUnit> kI,
        Per<O, PerUnit<DistanceUnit, TimeUnit>> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Meters, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public LinearPIDFeedback(Per<O, DistanceUnit> kP, Per<O, PerUnit<DistanceUnit, TimeUnit>> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Meters, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public LinearPIDFeedback(Per<O, DistanceUnit> kP) {
      super(kP);
    }

    public Pair<Measure<O>, Measure<DistanceUnit>> calculate(
        Distance measurement, Distance setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  public static class LinearVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, LinearVelocityUnit> {
    public LinearVelocityPIDFeedback(
        Per<O, LinearVelocityUnit> kP,
        Per<O, LinearVelocityUnit> kI,
        Per<O, LinearAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(MetersPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public LinearVelocityPIDFeedback(
        Per<O, LinearVelocityUnit> kP, Per<O, PerUnit<LinearVelocityUnit, TimeUnit>> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(MetersPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public LinearVelocityPIDFeedback(Per<O, LinearVelocityUnit> kP) {
      super(kP);
    }

    public Pair<Measure<O>, Measure<LinearVelocityUnit>> calculate(
        LinearVelocity measurement, LinearVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  public static class AngularPIDFeedback<O extends Unit> extends PIDFeedback<O, AngleUnit> {

    public AngularPIDFeedback(
        Per<O, AngleUnit> kP, Per<O, AngleUnit> kI, Per<O, AngularVelocityUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Radian, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public AngularPIDFeedback(Per<O, AngleUnit> kP, Per<O, AngularVelocityUnit> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Radian, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public AngularPIDFeedback(Per<O, AngleUnit> kP) {
      super(kP);
    }

    public Pair<Measure<O>, Measure<AngleUnit>> calculate(Angle measurement, Angle setpoint) {
      return super.calculate(measurement, setpoint);
    }

    public AngularPIDFeedback<O> withContinuousAngularInput() {
      super.withContinuousInput(Radian.of(-Math.PI), Radian.of(Math.PI));
      return this;
    }
  }

  public static class AngularVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, AngularVelocityUnit> {
    public AngularVelocityPIDFeedback(
        Per<O, AngularVelocityUnit> kP,
        Per<O, AngularVelocityUnit> kI,
        Per<O, AngularAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(RadiansPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public AngularVelocityPIDFeedback(
        Per<O, AngularVelocityUnit> kP, Per<O, AngularAccelerationUnit> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(RadiansPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    public AngularVelocityPIDFeedback(Per<O, AngularVelocityUnit> kP) {
      super(kP);
    }

    public Pair<Measure<O>, Measure<AngularVelocityUnit>> calculate(
        AngularVelocity measurement, AngularVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  public sealed interface Feedforward<O extends Unit, Q extends Unit>
      permits FlywheelFeedforward, ElevatorFeedforward, ArmFeedforward {
    public void logGains();
  }

  public static final class FlywheelFeedforward<O extends Unit>
      implements Feedforward<O, AngleUnit> {
    private final edu.wpi.first.math.controller.SimpleMotorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, AngularVelocityUnit> kV;
    final Per<O, AngularAccelerationUnit> kA;

    public FlywheelFeedforward(
        Measure<O> kS, Per<O, AngularVelocityUnit> kV, Per<O, AngularAccelerationUnit> kA) {
      outputUnit = kS.unit();
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      internalFeedforward =
          new edu.wpi.first.math.controller.SimpleMotorFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
    }

    public FlywheelFeedforward(Measure<O> kS) {
      outputUnit = kS.unit();
      this.kS = kS;
      this.kV = null;
      this.kA = null;
      internalFeedforward =
          new edu.wpi.first.math.controller.SimpleMotorFeedforward(
              kS.baseUnitMagnitude(), 0.0, 0.0);
    }

    @Override
    public void logGains() {
      Logger.recordOutput("Sim/kS", kS.baseUnitMagnitude());
      if (kV != null) {
        Logger.recordOutput("Sim/kV", kV.baseUnitMagnitude());
      }
      if (kA != null) {
        Logger.recordOutput("Sim/kA", kA.baseUnitMagnitude());
      }
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", Radian.name());
      Logger.recordOutput("Sim/feedforwardType", "Flywheel");
    }

    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
    }

    @SuppressWarnings("unchecked")
    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.baseUnitMagnitude(), nextGoalRate.baseUnitMagnitude()));
    }

    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
    }
  }

  public static final class ElevatorFeedforward<O extends Unit>
      implements Feedforward<O, DistanceUnit> {
    private final edu.wpi.first.math.controller.ElevatorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, VelocityUnit<DistanceUnit>> kV;
    final Per<O, AccelerationUnit<DistanceUnit>> kA;

    public ElevatorFeedforward(
        Measure<O> kS,
        Per<O, VelocityUnit<DistanceUnit>> kV,
        Per<O, AccelerationUnit<DistanceUnit>> kA) {
      outputUnit = kS.unit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ElevatorFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    @Override
    public void logGains() {
      Logger.recordOutput("Sim/kS", kS.baseUnitMagnitude());
      Logger.recordOutput("Sim/kV", kV.baseUnitMagnitude());
      Logger.recordOutput("Sim/kA", kA.baseUnitMagnitude());
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", Meters.name());
      Logger.recordOutput("Sim/feedforwardType", "Elevator");
    }

    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(LinearVelocity goalRate, LinearAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
    }

    @SuppressWarnings("unchecked")
    public Measure<O> calculate(LinearVelocity goalRate, LinearVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.baseUnitMagnitude(), nextGoalRate.baseUnitMagnitude()));
    }

    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(LinearVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
    }
  }

  public static final class ArmFeedforward<O extends Unit> implements Feedforward<O, AngleUnit> {
    private final edu.wpi.first.math.controller.ArmFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, VelocityUnit<AngleUnit>> kV;
    final Per<O, AccelerationUnit<AngleUnit>> kA;

    public ArmFeedforward(
        Measure<O> kS, Per<O, VelocityUnit<AngleUnit>> kV, Per<O, AccelerationUnit<AngleUnit>> kA) {
      outputUnit = kS.unit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ArmFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    @SuppressWarnings("unchecked")
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.baseUnitMagnitude(),
                  goalRate.baseUnitMagnitude(),
                  goalRateRate.baseUnitMagnitude()));
    }

    @SuppressWarnings("unchecked")
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  currentAngle.baseUnitMagnitude(),
                  goalRate.baseUnitMagnitude(),
                  nextGoalRate.baseUnitMagnitude()));
    }

    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.baseUnitMagnitude(), goalRate.baseUnitMagnitude()));
    }

    // TODO: Fill in.
    @Override
    public void logGains() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'logGains'");
    }
  }

  /** {@link TrapezoidProfile} */
  public static class TrapezoidProfile<DIM extends Unit> {
    private final edu.wpi.first.math.trajectory.TrapezoidProfile internalProfile;
    private final Measure<DIM> maxValue;
    private final Velocity<DIM> maxSlew;
    private final Acceleration<DIM> maxSlewSlew;

    public record State<DIM extends Unit>(Measure<DIM> value, Velocity<DIM> slew)
        implements StructSerializable {
      public static State<AngleUnit> of(Angle position, AngularVelocity velocity) {
        return new State<>(
            position, VelocityUnit.combine(Radian, Second).of(velocity.in(RadiansPerSecond)));
      }

      public static State<DistanceUnit> of(Distance position, LinearVelocity velocity) {
        return new State<>(
            position, VelocityUnit.combine(Meters, Second).of(velocity.in(MetersPerSecond)));
      }

      public static State<AngularVelocityUnit> of(
          AngularVelocity velocity, AngularAcceleration acceleration) {
        return new State<>(
            velocity,
            VelocityUnit.combine(RadiansPerSecond, Second)
                .of(acceleration.in(RadiansPerSecondPerSecond)));
      }

      public static State<LinearVelocityUnit> of(
          LinearVelocity velocity, LinearAcceleration acceleration) {
        return new State<>(
            velocity,
            VelocityUnit.combine(MetersPerSecond, Second)
                .of(acceleration.in(MetersPerSecondPerSecond)));
      }

      @SuppressWarnings("rawtypes")
      public static final Struct<State> struct = ProceduralStructGenerator.genRecord(State.class);
    }

    private TrapezoidProfile(
        Measure<DIM> maxValue, Velocity<DIM> maxSlew, Acceleration<DIM> maxSlewSlew) {
      this.maxValue = maxValue;
      this.maxSlew = maxSlew;
      this.maxSlewSlew = maxSlewSlew;
      internalProfile =
          new edu.wpi.first.math.trajectory.TrapezoidProfile(
              new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                  maxSlew.baseUnitMagnitude(), maxSlewSlew.baseUnitMagnitude()));
    }

    public static TrapezoidProfile<AngleUnit> forAngle(
        AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
      final VelocityUnit<AngleUnit> vu = VelocityUnit.combine(Radian, Second);
      return new TrapezoidProfile<>(
          Radian.of(10000000000.0),
          vu.of(maxVelocity.in(RadiansPerSecond)),
          AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(RadiansPerSecondPerSecond)));
    }

    public static TrapezoidProfile<DistanceUnit> forDistance(
        LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
      final VelocityUnit<DistanceUnit> vu = VelocityUnit.combine(Meters, Second);
      return new TrapezoidProfile<>(
          Meters.of(10000000000.0),
          vu.of(maxVelocity.in(MetersPerSecond)),
          AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(MetersPerSecondPerSecond)));
    }

    public static TrapezoidProfile<AngularVelocityUnit> forAngularVelocity(
        AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
      final VelocityUnit<AngularVelocityUnit> vu = VelocityUnit.combine(RadiansPerSecond, Second);
      return new TrapezoidProfile<>(
          maxVelocity,
          vu.of(maxAcceleration.in(RadiansPerSecondPerSecond)),
          AccelerationUnit.combine(vu, Second).of(10000000.0));
    }

    public static TrapezoidProfile<LinearVelocityUnit> forLinearVelocity(
        LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
      final VelocityUnit<LinearVelocityUnit> vu = VelocityUnit.combine(MetersPerSecond, Second);
      return new TrapezoidProfile<>(
          maxVelocity,
          vu.of(maxAcceleration.in(MetersPerSecondPerSecond)),
          AccelerationUnit.combine(vu, Second).of(10000000.0));
    }

    public void logConstraints(EpilogueBackend logger) {
      logger.log("maxValue", maxValue.baseUnitMagnitude());
      logger.log("maxSlew", maxSlew.baseUnitMagnitude());
      logger.log("maxSlewSlew", maxSlewSlew.baseUnitMagnitude());
      logger.log("constraintUnit", maxValue.unit().name());
    }

    @SuppressWarnings("unchecked")
    public State<DIM> calculate(State<DIM> current, State<DIM> goal, Time deltaTime) {
      var internalState =
          internalProfile.calculate(
              deltaTime.baseUnitMagnitude(),
              new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                  current.value.baseUnitMagnitude(), current.slew.baseUnitMagnitude()),
              new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                  goal.value.baseUnitMagnitude(), goal.slew.baseUnitMagnitude()));

      Measure<DIM> value = (Measure<DIM>) current.value.unit().of(internalState.position);
      Velocity<DIM> slew = VelocityUnit.combine(value.unit(), Second).of(internalState.velocity);
      if (MeasureMath.abs(value).gt(maxValue)) {
        value = MeasureMath.clamp(value, maxValue);
        return new State<>(value, (Velocity<DIM>) slew.unit().zero());
      } else {
        return new State<>(value, slew);
      }
    }
  }
}
