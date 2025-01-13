package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.extras.sim.SimSwerveModule;
import frc.robot.extras.sim.utils.GearRatio;
import frc.robot.extras.util.CTREUtil.FusedTalonFxSimController;
import frc.robot.extras.util.CTREUtil.TalonFXSimController;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule extends PhysicalModule {
  private final SimSwerveModule moduleSimulation;

  FusedTalonFxSimController turnSim
  TalonFXSimController driveSim;
  // TODO: retune possibly most likely
  private final PIDController drivePID = new PIDController(.27, 0, 0);
  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 1.5);

  private final Constraints turnConstraints =
      new Constraints(
          ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND,
          ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED);
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(18, 0, 0, turnConstraints);
  private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0, 0, 0);

  // Drive signals
  private Angle drivePosition;
  private AngularVelocity driveVelocity;
  private Voltage driveAppliedVolts;
  private Current driveCurrentAmps;

  // Turn signals
  private Rotation2d turnAbsolutePosition;
  private AngularVelocity turnVelocity;
  private Voltage turnAppliedVolts;
  private Current turnCurrentAmps;

  public SimulatedModule(ModuleConfig config, SimSwerveModule moduleSimulation) {
    super(config);
    this.moduleSimulation = moduleSimulation;
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
    turnSim = new FusedTalonFxSimController(getTurnMotor().getSimState(), getTurnEncoder().getSimState(), GearRatio.reduction(ModuleConstants.TURN_GEAR_RATIO));
    driveSim =  new TalonFXSimController(getDriveMotor().getSimState(), config.driveReversed());
    drivePosition = moduleSimulation.outputs().drive().position();
    driveVelocity = moduleSimulation.outputs().drive().velocity();
    driveAppliedVolts = moduleSimulation.inputs().drive().statorVoltage();
    driveCurrentAmps = moduleSimulation.inputs().drive().statorCurrent();
    turnAbsolutePosition = new Rotation2d(moduleSimulation.outputs().steer().position());
    turnVelocity = moduleSimulation.outputs().steer().velocity();
    turnAppliedVolts = moduleSimulation.inputs().steer().statorVoltage();
    turnCurrentAmps = moduleSimulation.inputs().steer().statorCurrent();
  }

  // TODO: maybe add supply?
  @Override
  public void updateInputs(ModuleInputs inputs) {
    // TODO: add drive accel
    inputs.drivePosition = drivePosition.in(Rotations);
    inputs.driveVelocity = driveVelocity.in(RotationsPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts.in(Volts);
    inputs.driveCurrentAmps = driveCurrentAmps.in(Amps);

    inputs.turnAbsolutePosition = turnAbsolutePosition;
    inputs.turnVelocity = turnVelocity.in(RotationsPerSecond);

    inputs.turnAppliedVolts = turnAppliedVolts.in(Volts);
    inputs.turnCurrentAmps = turnCurrentAmps.in(Amps);

    inputs.isConnected = true;
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    volts = driveAppliedVolts; // TODO: see if this works
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    volts = turnAppliedVolts; // TODO: see if this works
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // driveSim.run(null, driveAppliedVolts, desiredState)
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

            // moduleSimulation.state().speedMetersPerSecond;
    setDriveVoltage(
              Volts.of(drivePID.calculate(
                    moduleSimulation.outputs().drive().velocity().in(RotationsPerSecond),
                    desiredDriveRPS)
            + (driveFF.calculate(desiredDriveRPS))));
    setTurnVoltage(
        Volts.of(
                turnPID.calculate(
                    getTurnRotations(),
                    desiredState.angle.getRotations()) 
                    + (turnFF.calculate(turnPID.getSetpoint().velocity))));
  }

  @Override
  public double getTurnRotations() {
    return turnAbsolutePosition.getRotations();
  }

  @Override
  public void stopModule() {
    setDriveVoltage(Volts.zero());
    setTurnVoltage(Volts.zero());
  }
}
