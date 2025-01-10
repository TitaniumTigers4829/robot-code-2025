package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.extras.sim.SimSwerveModule;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final SimSwerveModule moduleSimulation;

  private final PIDController drivePID = new PIDController(.27, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 1.5);

  private final Constraints turnConstraints =
      new Constraints(
          ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND,
          ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED);
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(18, 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0, 0, 0);

  public SimulatedModule(SimSwerveModule moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // TODO: maybe add supply and stator?
  @Override
  public void updateInputs(ModuleInputs inputs) {
    inputs.drivePosition = moduleSimulation.outputs().drive().position().in(Rotations);
    inputs.driveVelocity = moduleSimulation.outputs().drive().velocity().in(RotationsPerSecond);

    inputs.driveAppliedVolts = moduleSimulation.inputs().drive().statorVoltage().in(Volts);
    inputs.driveCurrentAmps = moduleSimulation.inputs().drive().statorCurrent().in(Amps);

    inputs.turnAbsolutePosition = new Rotation2d(moduleSimulation.outputs().steer().position().in(Radians));
    inputs.turnVelocity = moduleSimulation.outputs().steer().velocity().in(RotationsPerSecond);

    inputs.turnAppliedVolts = moduleSimulation.inputs().steer().statorVoltage().in(Volts);
    inputs.turnCurrentAmps = moduleSimulation.inputs().steer().statorCurrent().in(Amps);

    // inputs.odometrySteerPositions = moduleSimulation.getCachedTurnAbsolutePositions();

    // inputs.odometryTimestamps = OdometryTimestampsSim.getTimestamps();

    inputs.isConnected = true;
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    // moduleSimulation.state().
    // moduleSimulation.(volts);
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    // moduleSimulation.requestTurnVoltageOut(volts);
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

            // moduleSimulation.state().speedMetersPerSecond;
    // moduleSimulation.requestDriveVoltageOut(
    //     Volts.of(
    //             drivePID.calculate(
    //                 moduleSimulation.outputs().drive().velocity().in(RotationsPerSecond)
    //                     * ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
    //                 moduleSimulation.state().speedMetersPerSecond * ModuleConstants.DRIVE_TO_METERS_PER_SECOND))
    //         .plus(Volts.of(driveFF.calculate(desiredDriveRPS))));
    // moduleSimulation.requestTurnVoltageOut(
    //     Volts.of(
    //             turnPID.calculate(
    //                 moduleSimulation.getTurnAbsolutePosition().getRotations(),
    //                 desiredState.angle.getRotations()))
    //         .plus(Volts.of(turnFF.calculate(turnPID.getSetpoint().velocity))));
  }

  @Override
  public double getTurnRotations() {
    return moduleSimulation.getTurnAbsolutePosition().getRotations();
  }

  @Override
  public void stopModule() {
    moduleSimulation.requestDriveVoltageOut(Volts.of(0));
    moduleSimulation.requestTurnVoltageOut(Volts.of(0));
  }
}
