package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.util.GeomUtil;
import frc.robot.extras.util.ThreadManager;
import frc.robot.extras.vision.LimelightHelpers;
import frc.robot.extras.vision.LimelightHelpers.PoseEstimate;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.concurrent.atomic.AtomicReferenceArray;

/**
 * This class is the implementation of the VisionInterface for the physical robot. It uses the
 * ThreadManager to make threads to run the code for processing the vision data from the limelights
 * asynchonously.
 *
 * @author Jack
 * @author Ishan
 */
public class PhysicalVision implements VisionInterface {

  private Pose2d odometryPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;

  /**
   * The pose estimates from the limelights in the following order (BACK, FRONT_LEFT, FRONT_RIGHT)
   */
  private final AtomicReferenceArray<MegatagPoseEstimate> limelightEstimates =
      new AtomicReferenceArray<>(
          new MegatagPoseEstimate[] {
            new MegatagPoseEstimate(), new MegatagPoseEstimate(), new MegatagPoseEstimate()
          });

  private final ThreadManager threadManager = new ThreadManager(Limelight.values().length);

  public PhysicalVision() {
    for (Limelight limelight : Limelight.values()) {
      // Start a vision input task for each Limelight
      threadManager.startTask(
          limelight.getName(),
          () -> checkAndUpdatePose(limelight),
          VisionConstants.THREAD_SLEEP_MS);
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // Combine inputs into the main inputs object
    for (Limelight limelight : Limelight.values()) {
      inputs.isLimelightConnected[limelight.getId()] = isLimelightConnected(limelight);

      inputs.limelightSeesAprilTags[limelight.getId()] = canSeeAprilTags(limelight);

      inputs.limelightTargets[limelight.getId()] = getNumberOfAprilTags(limelight);

      inputs.limelightLatencies[limelight.getId()] = getLatencySeconds(limelight);
      inputs.limelightAprilTagDistances[limelight.getId()] =
          getLimelightAprilTagDistance(limelight);
      inputs.limelightTimestamps[limelight.getId()] = getTimeStampSeconds(limelight);
      inputs.limelightAmbiguities[limelight.getId()] = getAmbiguity(limelight);

      inputs.limelightCalculatedPoses[limelight.getId()] = getPoseFromAprilTags(limelight);
    }
  }

  @Override
  public boolean canSeeAprilTags(Limelight limelight) {
    // First checks if it can see an april tag, then checks if it is fully in frame as
    // the limelight can see an april tag but not have it fully in frame, leading to
    // inaccurate pose estimates
    if (LimelightHelpers.getTV(limelight.getName())) {
      return Math.abs(LimelightHelpers.getTX(limelight.getName())) <= limelight.getAccurateFOV();
    }
    return false;
  }

  @Override
  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    Pose2d currentPose = limelightEstimates.get(limelight.getId()).fieldToCamera;
    return currentPose != null ? currentPose : odometryPose;
  }

  @Override
  public int getNumberOfAprilTags(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).tagCount;
  }

  @Override
  public double getTimeStampSeconds(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).timestampSeconds / 1000.0;
  }

  @Override
  public double getLatencySeconds(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).latency / 1000.0;
  }

  @Override
  public double getLimelightAprilTagDistance(Limelight limelight) {
    if (canSeeAprilTags(limelight)) {
      return limelightEstimates.get(limelight.getId()).avgTagDist;
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  @Override
  public double getAmbiguity(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).ambiguity;
  }

  @Override
  public void setOdometryInfo(
      double headingDegrees, double headingRateDegrees, Pose2d odometryPose) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
    this.odometryPose = odometryPose;
  }

  /**
   * Gets the JSON dump from the specified limelight and puts it into a PoseEstimate object, which
   * is then placed into its corresponding spot in the limelightEstimates array.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void enabledPoseUpdate(Limelight limelight) {
    PoseEstimate megatag1Estimate = getMegaTag1PoseEstimate(limelight);
    PoseEstimate megatag2Estimate = getMegaTag2PoseEstimate(limelight);
    if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE
        && isWithinFieldBounds(megatag2Estimate.pose)) {
      LimelightHelpers.SetRobotOrientation(
          limelight.getName(), headingDegrees, headingRateDegreesPerSecond, 0, 0, 0, 0);
      limelightEstimates.set(
          limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag2Estimate));
    } else if (isWithinFieldBounds(megatag1Estimate.pose)) {
      limelightEstimates.set(
          limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag1Estimate));
    }
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void disabledPoseUpdate(Limelight limelight) {
    PoseEstimate megatag1PoseEstimate = getMegaTag1PoseEstimate(limelight);
    limelightEstimates.set(
        limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag1PoseEstimate));
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void updatePoseEstimate(Limelight limelight) {
    if (DriverStation.isEnabled()) {
      enabledPoseUpdate(limelight);
    } else {
      disabledPoseUpdate(limelight);
    }
  }

  /**
   * Checks if there is a large discrepancy between the MegaTag1 and MegaTag2 estimates.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the discrepancy is larger than the defined threshold, false otherwise
   */
  public boolean isLargeDiscrepancyBetweenMegaTag1And2(
      Limelight limelight, PoseEstimate mt1, PoseEstimate mt2) {
    return !GeomUtil.areTranslationsWithinThreshold(
            VisionConstants.MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD,
            mt1.pose.getTranslation(),
            mt2.pose.getTranslation())
        || !GeomUtil.areRotationsWithinThreshold(
            VisionConstants.MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD,
            mt1.pose.getRotation(),
            mt2.pose.getRotation());
  }

  /**
   * Gets the MegaTag1 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The MegaTag1 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag1PoseEstimate(Limelight limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());
  }

  /**
   * Gets the MegaTag2 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The MegaTag2 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag2PoseEstimate(Limelight limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
  }

  /**
   * Checks if the MT1 and MT2 pose estimate exists and whether it is within the field
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the pose estimate exists within the field and the pose estimate is not null
   */
  public boolean isValidPoseEstimate(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).fieldToCamera != null
        && isWithinFieldBounds(limelightEstimates.get(limelight.getId()).fieldToCamera);
  }

  /**
   * Checks whether the pose estimate for MT1 or MT2 is within the field
   *
   * @param megaTag1Estimate the MT1 pose estimate to check
   * @param megaTag2Estimate the MT2 pose estimate to check
   */
  private boolean isWithinFieldBounds(Pose2d poseEstimate) {
    return ((poseEstimate.getX() > 0 && poseEstimate.getX() <= FieldConstants.FIELD_LENGTH_METERS)
        && (poseEstimate.getY() > 0 && poseEstimate.getY() <= FieldConstants.FIELD_WIDTH_METERS));
  }

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(Limelight limelight) {
    return LimelightHelpers.getLimelightNTTable(limelight.getName()).containsKey("tv");
  }

  @Override
  public boolean isValidMeasurement(Limelight limelight) {
    return isValidPoseEstimate(limelight);
    // !isTeleporting(limelight);.
    //  && isConfident(limelight);
  }

  /**
   * Checks if the robot is teleporting based on the pose estimate from the limelight
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the robot is teleporting, false otherwise
   */
  private boolean isTeleporting(Limelight limelight) {
    return GeomUtil.arePosesWithinThreshold(
        VisionConstants.MAX_TRANSLATION_DELTA_METERS,
        VisionConstants.MAX_ROTATION_DELTA_DEGREES,
        getPoseFromAprilTags(limelight));
  }

  /**
   * Checks if the limelight is confident in its pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight is confident in its pose estimate, false otherwise
   */
  private boolean isConfident(Limelight limelight) {
    return getAmbiguity(limelight) >= VisionConstants.MIN_CONFIDENCE_THRESHOLD;
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void checkAndUpdatePose(Limelight limelight) {
    if (isLimelightConnected(limelight) && canSeeAprilTags(limelight)) {
      updatePoseEstimate(limelight);
    } else {
      limelightEstimates.set(limelight.getId(), new MegatagPoseEstimate());
    }
  }

  /**
   * Sets the AtomicBoolean 'runningThreads' to false for the specified limelight. Stops the thread
   * for the specified limelight.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void stopLimelightThread(Limelight limelight) {
    threadManager.stopThread(limelight.getName());
  }

  /** Shuts down all the threads. */
  public void endAllThreads() {
    threadManager.shutdownAllThreads();
  }
}
