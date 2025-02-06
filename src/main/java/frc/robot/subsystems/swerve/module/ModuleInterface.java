package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleInterface {

  @AutoLog
  class ModuleInputs {
    public boolean isConnected = false;

    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double drivePosition = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  /**
   * Updates the inputs created in ModuleInputs
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(ModuleInputs inputs) {}

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  default void setDesiredState(SwerveModuleState desiredState) {}

  /**
   * Sets voltage of the drive motor for the module.
   *
   * @param voltage Voltage to set the drive motor to.
   */
  default void setDriveVoltage(Voltage voltage) {}

  /**
   * Sets voltage of the turn motor for the module.
   *
   * @param voltage Voltage to set the turn motor to.
   */
  default void setTurnVoltage(Voltage voltage) {}

  /** Stops the motors in the module. */
  default void stopModule() {}

  /**
   * Gets the current turn position of the module.
   *
   * @return The current turn position of the module.
   */
  default double getTurnRotations() {
    return 0.0;
  }

  default double getDrivePosition() {
    return 0.0;
  }
}
