package frc.robot.extras.swerve.setpointGen;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;

public final class AdvancedSwerveModuleState extends SwerveModuleState {
  public double steerVelocityFF;
  public double driveAccelerationFF;

  public AdvancedSwerveModuleState(
      double speedMetersPerSecond,
      Rotation2d angle,
      double steerVelocityFF,
      double driveAccelerationFF) {
    super(speedMetersPerSecond, angle);
    this.steerVelocityFF = steerVelocityFF;
    this.driveAccelerationFF = driveAccelerationFF;
  }

  // todo: implement custom struct
  public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

  public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
    return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0, 0.0);
  }
}
