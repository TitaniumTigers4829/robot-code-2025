package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.AutoLog;

public interface VisionInterface {

  /**
   * This class is used to store the inputs for the vision subsystem. Each of its fields that are 
   * arrays have a length equal to the number of Limelights on the robot. Each index of the arrays
   * corresponds to a different Limelight (0 is back, 1 is front left, 2 is front right). 
   */
  @AutoLog
  class VisionInputs {
    /**
     * This array stores whether each Limelight is connected to the robot.
     */
    public boolean[] isLimelightConnected = new boolean[Limelight.values().length];
    /**
     * This array stores MegatagPoseEstimates for each Limelight.
     */
    public MegatagPoseEstimate[] limelightMegatagPoses =
        new MegatagPoseEstimate[Limelight.values().length];
    /**
     * This array stores the latencies in seconds of each Limelight.
     */
    public double[] limelightLatencies = new double[Limelight.values().length];
    /**
     * This array stores the number of april tags each Limelight sees.
     */
    public int[] limelightTargets = new int[Limelight.values().length];
    /**
     * This array stores whether each Limelight sees any April Tags.
     */
    public boolean[] limelightSeesAprilTags = new boolean[Limelight.values().length];
    /**
     * This array stores the poses calculated from the April Tags seen by each Limelight.
     */
    public Pose2d[] limelightCalculatedPoses = new Pose2d[Limelight.values().length];
    /**
     * This array stores the distances in meters to the April Tags seen by each Limelight.
     */
    public double[] limelightAprilTagDistances = new double[Limelight.values().length];
    /**
     * This array stores the timestamps in seconds of the data from each Limelight.
     */
    public double[] limelightTimestamps = new double[Limelight.values().length];

    /**
     * This stores the last seen pose of any Limelight that most recently saw a target.
     * This is primarily used if a driver wants to reset the robot's pose to what the limelights are seeing.
     */
    public Pose2d limelightLastSeenPose = new Pose2d();
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
