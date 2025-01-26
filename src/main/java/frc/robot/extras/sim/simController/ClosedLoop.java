package frc.robot.extras.sim.simController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.extras.sim.simController.UnitSafeControl.ArmFeedforward;
import frc.robot.extras.sim.simController.UnitSafeControl.ElevatorFeedforward;
import frc.robot.extras.sim.simController.UnitSafeControl.Feedforward;
import frc.robot.extras.sim.simController.UnitSafeControl.FlywheelFeedforward;
import frc.robot.extras.sim.simController.UnitSafeControl.PIDFeedback;
import frc.robot.extras.sim.simController.UnitSafeControl.TrapezoidProfile;
import frc.robot.extras.sim.simController.UnitSafeControl.TrapezoidProfile.State;
import java.util.Optional;
import java.util.function.Function;

public class ClosedLoop<OUTPUT extends Unit, INPUT extends Unit, INPUT_DIMENSION extends Unit> {
  private final PIDFeedback<OUTPUT, INPUT> feedback;
  private final Feedforward<OUTPUT, INPUT_DIMENSION> feedforward;
  private final Optional<TrapezoidProfile<INPUT>> optTrapezoidProfile;
  private final boolean useFeedbackSign;
  private final Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance;

  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      Feedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<TrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    this.angleToDistance = angle -> Meters.zero();
  }

  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      Feedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<TrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    this.angleToDistance = angleToDistance;
  }

  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      Feedforward<VoltageUnit, DistanceUnit> feedforward,
      TrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      Feedforward<VoltageUnit, DistanceUnit> feedforward,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false, angleToDistance);
  }

  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      Feedforward<CurrentUnit, DistanceUnit> feedforward,
      TrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      Feedforward<CurrentUnit, DistanceUnit> feedforward,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.empty(), useFeedbackSign, angleToDistance);
  }

  public boolean isVelocity() {
    return feedback.getInputUnit() instanceof PerUnit;
  }

  private State<DistanceUnit> angleToDistanceState(State<AngleUnit> state) {
    final VelocityUnit<DistanceUnit> velocityUnit = VelocityUnit.combine(Meters, Seconds);
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    return new State<>(
        angleToDistance.apply(state.value()),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  private State<LinearVelocityUnit> angularVelocityToLinearVelocityState(
      State<AngularVelocityUnit> state) {
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    final VelocityUnit<LinearVelocityUnit> velocityUnit =
        VelocityUnit.combine(MetersPerSecond, Seconds);
    return new State<>(
        MetersPerSecond.of(state.value().baseUnitMagnitude() * ratio),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runPosition(
      Angle position, State<AngleUnit> state, State<AngleUnit> goal, Time dt) {
    if (isVelocity()) {
      throw new UnsupportedOperationException("Velocity not supported for `runAngle`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angleToDistanceState(state),
          (State<INPUT>) angleToDistanceState(goal),
          dt);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position, (State<INPUT>) state, (State<INPUT>) goal, dt);
    }
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runVelocity(
      Angle position, State<AngularVelocityUnit> state, State<AngularVelocityUnit> goal, Time dt) {
    if (!isVelocity()) {
      throw new UnsupportedOperationException("Position not supported for `runVelocity`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angularVelocityToLinearVelocityState(state),
          (State<INPUT>) angularVelocityToLinearVelocityState(goal),
          dt);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position, (State<INPUT>) state, (State<INPUT>) goal, dt);
    }
  }

  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> run(
      Measure<INPUT_DIMENSION> position, State<INPUT> state, State<INPUT> goal, Time dt) {
    boolean isVelocity = feedback.getInputUnit() instanceof PerUnit;
    if (optTrapezoidProfile.isPresent()) {
      var trapezoidProfile = optTrapezoidProfile.get();
      State<INPUT> step = trapezoidProfile.calculate(state, goal, dt);
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      if (feedforward.getClass().equals(FlywheelFeedforward.class)) {
        var flywheelFF = (FlywheelFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? flywheelFF.calculate(
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : flywheelFF.calculate(RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? elevatorFF.calculate(
                    MetersPerSecond.of(step.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : elevatorFF.calculate(MetersPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : armFF.calculate(
                    Radians.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      var fbResult = feedback.calculate(state.value(), step.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      return feedbackOutput.plus(feedforwardOutput);
    } else {
      // Measure<OUTPUT> feedbackOutput = feedback.calculate(state.value(), goal.value());
      var fbResult = feedback.calculate(state.value(), goal.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      double velocitySign =
          useFeedbackSign
              ? 0.00001 * Math.signum(feedbackOutput.baseUnitMagnitude())
              : 0.00001
                  * Math.signum(
                      goal.value().baseUnitMagnitude() - state.value().baseUnitMagnitude());
      if (feedforward.getClass().equals(FlywheelFeedforward.class)) {
        var flywheelFF = (FlywheelFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(MetersPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      return feedbackOutput.plus(feedforwardOutput);
    }
  }
}
