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
    /** This array stores whether each Limelight is connected to the robot. */
    public boolean[] isLimelightConnected = new boolean[Limelight.values().length];

    /** This array stores MegatagPoseEstimates for each Limelight. */
    public MegatagPoseEstimate[] limelightMegatagPoses =
        new MegatagPoseEstimate[Limelight.values().length];

    /** This array stores the latencies in seconds of each Limelight. */
    public double[] limelightLatencies = new double[Limelight.values().length];

    /** This array stores the number of april tags each Limelight sees. */
    public int[] limelightTargets = new int[Limelight.values().length];

    /** This array stores whether each Limelight sees any April Tags. */
    public boolean[] limelightSeesAprilTags = new boolean[Limelight.values().length];

    /** This array stores the poses calculated from the April Tags seen by each Limelight. */
    public Pose2d[] limelightCalculatedPoses = new Pose2d[Limelight.values().length];

    /**
     * This array stores the average distances in meters to the April Tags seen by each Limelight.
     */
    public double[] limelightAprilTagDistances = new double[Limelight.values().length];

    /** This array stores the timestamps in seconds of the data from each Limelight. */
    public double[] limelightTimestamps = new double[Limelight.values().length];
  }

  /**
   * Updates the inputs for the vision subsystem using VisionInputs
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(VisionInputs inputs) {}

  /**
   * Gets the current latency of the Limelight in seconds. This latency is the time it takes for the
   * Limelight to process an image and send the data to the robot.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The current latency of the Limelight
   */
  default double getLatencySeconds(Limelight limelight) {
    return 0.0;
  }

  /**
   * Gets the current timestamp of the Limelight in seconds. This timestamp is the time at which the
   * data was received from the Limelight.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The current timestamp of the Limelight
   */
  default double getTimeStampSeconds(Limelight limelight) {
    return 0.0;
  }

  /**
   * Checks if the specified limelight can fully see one or more April Tag.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return true if the Limelight can see any april tags, and if it is within its field of view
   */
  default boolean canSeeAprilTags(Limelight limelight) {
    return false;
  }

  /**
   * Gets the average distance between the specified limelight and the April Tags it sees
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return the average distance between the robot and the April Tag(s) in meters
   */
  default double getLimelightAprilTagDistance(Limelight limelight) {
    return 0.0;
  }

  /**
   * Gets how many april tags the limelight can see.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The current number of April Tags of the limelight sees
   */
  default int getNumberOfAprilTags(Limelight limelight) {
    return 0;
  }

  /**
   * Gets the pose of the robot calculated by the specified limelight via any April Tags it sees
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return the pose of the robot, if the limelight can't see any April Tags, it will return 0 for
   *     x, y, and theta
   */
  default Pose2d getPoseFromAprilTags(Limelight limelight) {
    return null;
  }

  /**
   * Sets the heading and heading rate of the robot, this is used for deciding between MegaTag 1 and
   * 2 for pose estimation.
   *
   * @param headingDegrees the angle the robot is facing in degrees (0 degrees facing the red
   *     alliance)
   * @param headingRateDegreesPerSecond the rate the robot is rotating, CCW positive
   */
  default void setOdometryInfo(double headingDegrees, double headingRateDegreesPerSecond, Pose2d odometryPose) {}
}
