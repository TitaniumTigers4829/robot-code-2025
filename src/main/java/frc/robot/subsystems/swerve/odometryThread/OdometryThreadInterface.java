package frc.robot.subsystems.swerve.odometryThread;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for odometry logging. Fields annotated with @AutoLog will be captured by AdvantageKit.
 */
public interface OdometryThreadInterface {
  int MODULE_COUNT = 4; // Typically four swerve modules

  @AutoLog
  class OdometryInputs {
    public double cycleTimeMillis = 0.0;
    public double accelMagnitude = 0.0;
    public double gyroAngle = 0.0;
    
    // Drive position (meters) for each module.
    public double[] drivePositions = new double[MODULE_COUNT];
    // Drive velocity (meters per second) for each module.
    public double[] driveVelocities = new double[MODULE_COUNT];

    // Turn absolute angle (in degrees) for each module.
    public double[] turnAngles = new double[MODULE_COUNT];
    // Turn velocity (degrees per second) for each module.
    public double[] turnVelocities = new double[MODULE_COUNT];
  }

  /**
   * Updates the odometry inputs with sensor values.
   *
   * @param inputs The odometry inputs to update.
   */
  default void updateInputs(OdometryInputs inputs) {}
}
