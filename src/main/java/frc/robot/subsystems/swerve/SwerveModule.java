package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  private final ModuleInterface io;
  private final String name;
  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private final Alert hardwareFaultAlert;

  public SwerveModule(ModuleInterface io, String name) {
    super("Module-" + name);
    this.io = io;
    this.name = name;
    this.hardwareFaultAlert =
        new Alert("Module-" + name + " Hardware Fault", Alert.AlertType.kError);
    this.hardwareFaultAlert.set(false);

    // We have essentially a nested subsystem here, so we need to unregister it with the scheduler
    CommandScheduler.getInstance().unregisterSubsystem(this);
  }

  /** 
   * Updates the module's odometry inputs. 
   */
  public void updateOdometryInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module-" + name, inputs);
    this.hardwareFaultAlert.set(!inputs.isConnected);
  }

  /** 
   * Sets the drive voltage of the module. 
   * 
   * @param volts the voltage to set the drive motor to
   * */
  public void setVoltage(Voltage volts) {
    io.setDriveVoltage(volts);
    io.setTurnVoltage(Volts.zero());
  }

  /**
   * Gets the drive voltage of the module.
   * 
   * @return the drive voltage of the module
   */
  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  /**
   * Gets the drive velocity of the module.
   * 
   * @return the drive velocity of the module
   */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocity;
  }

  /**
   * Sets the desired state of the module. It optimizes this meaning that it will
   * adjust the turn angle to be the shortest path to the desired angle. So rather
   * than turning 170 degrees CW it will turn 10 degrees CCW and invert the motor.
   * 
   * @param state
   */
  public void setOptimizedDesiredState(SwerveModuleState state) {
    state.optimize(getTurnRotation());
    if (state.speedMetersPerSecond < 0.01) {
      io.stopModule();
    }
    io.setDesiredState(state);
    Logger.recordOutput("Drive/desired turn angle", state.angle.getRotations());
  }

  /**
   * Gets the turn angle of the module.
   * @return the turn angle of the module 0 being forward, CCW being positive
   */
  public Rotation2d getTurnRotation() {
    return inputs.turnAbsolutePosition;
  }

  /**
   * Gets the turn velocity of the module.
   * @return the turn velocity in rotations per second
   */
  public double getTurnVelocity() {
    return inputs.turnVelocity;
  }

  /**
   * Gets the drive position of the module in meters.
   * @return the drive position in meters
   */
  public double getDrivePositionMeters() {
    return ModuleConstants.WHEEL_CIRCUMFERENCE_METERS * inputs.drivePosition;
  }

  /** 
   * Gets the drive velocity of the module in meters per second.
   * @return the drive velocity in meters per second
   */
  public double getDriveVelocityMetersPerSec() {
    return ModuleConstants.WHEEL_CIRCUMFERENCE_METERS * inputs.driveVelocity;
  }

  /**
   * Gets the measured state of the module consisting of the velocity and angle.
   * @return a SwerveModuleState object containing velocity and angle
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnRotation());
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is
   * rotated.
   *
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getTurnRotation());
  }
}
