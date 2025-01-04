package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {
  @AutoLog
  class VisionInputs {
    public boolean[] isLimelightConnected = new boolean[Limelight.values().length];

    public MegatagPoseEstimate[] limelightMegatagPoses =
        new MegatagPoseEstimate[Limelight.values().length];
    public double[] limelightLatencies = new double[Limelight.values().length];
    public int[] limelightTargets = new int[Limelight.values().length];
    public boolean[] limelightSeesAprilTags = new boolean[Limelight.values().length];

    public Pose2d[] limelightCalculatedPoses = new Pose2d[Limelight.values().length];
    public Pose2d limelightLastSeenPose = new Pose2d();
    public double[] limelightAprilTagDistances = new double[Limelight.values().length];

    public double[] limelightTimestamps = new double[Limelight.values().length];
  }

  /**
   * Updates the inputs for the vision subsystem using VisionInputs
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(VisionInputs inputs) {}

  /**
   * @param limelight The Limelight to retrieve the latency data from
   * @return The current latency of the Limelight
   */
  default double getLatencySeconds(Limelight limelight) {
    return 0.0;
  }

  /**
   * @param limelight The Limelight to retrieve the timestamp data from
   * @return The current timestamp of the Limelight
   */
  default double getTimeStampSeconds(Limelight limelight) {
    return 0.0;
  }

  /**
   * @param limelight The Limelight to retrieve the target data from
   * @return Whether the Limelight can see any targets, and if it is within its field of view
   */
  default boolean canSeeAprilTags(Limelight limelight) {
    return false;
  }

  /**
   * @param limelight The Limelight to retrieve the distance data from
   * @return The current April Tag distance from the Limelight
   */
  default double getLimelightAprilTagDistance(Limelight limelight) {
    return 0.0;
  }

  /**
   * @param limelight The Limelight to retrieve the data from
   * @return The current number of April Tags of the Limelight
   */
  default int getNumberOfAprilTags(Limelight limelight) {
    return 0;
  }

  /**
   * @param limelight The Limelight to retrieve the pose data from
   * @return The current pose of the Limelight
   */
  default Pose2d getPoseFromAprilTags(Limelight limelight) {
    return null;
  }

  /**
   * Sets the heading information of the robot, used with MT2
   *
   * @param headingDegrees The heading of the robot in degrees
   * @param headingRateDegrees The rate of change of the heading of the robot in degrees
   */
  default void setHeadingInfo(double headingDegrees, double headingRateDegrees) {}

  /**
   * @return The last seen pose of any Limelight that most recently saw a target
   */
  default Pose2d getLastSeenPose() {
    return null;
  }
}
