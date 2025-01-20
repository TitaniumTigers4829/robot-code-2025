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
      inputs.limelightLatencies[limelight.getId()] = getLatencySeconds(limelight);
      inputs.limelightTargets[limelight.getId()] = getNumberOfAprilTags(limelight);
      inputs.limelightSeesAprilTags[limelight.getId()] = canSeeAprilTags(limelight);
      inputs.limelightAprilTagDistances[limelight.getId()] =
          getLimelightAprilTagDistance(limelight);
      inputs.limelightCalculatedPoses[limelight.getId()] = getPoseFromAprilTags(limelight);
      inputs.limelightTimestamps[limelight.getId()] = getTimeStampSeconds(limelight);
    }
  }

  @Override
  public boolean canSeeAprilTags(Limelight limelight) {
    // First checks if it can see an april tag, then checks if it is fully in frame as
    // the limelight can see an april tag but not have it fully in frame, leading to
    // inaccurate pose estimates
    if (isValidID(limelight, getNumberOfAprilTags(limelight))) {
      return Math.abs(LimelightHelpers.getTX(limelight.getName())) <= limelight.getAccurateFOV();
    }
    return false;
  }

  @Override
  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).fieldToCamera;
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
    // if (!canSeeAprilTags(limelight)) {
    //   limelightEstimates[limelight.getId()] = new MegatagPoseEstimate();
    // } else {
    // if (canSeeAprilTags(limelight)) {
    // limelightEstimates[limelight.getId()] = new MegatagPoseEstimate();

    // && isValidPoseEstimate(limelight, megatag1Estimate, megatag2Estimate)) {
    // if (isLargeDiscrepancyBetweenMegaTag1And2(limelight, megatag1Estimate, megatag2Estimate)
    //     && getLimelightAprilTagDistance(limelight)
    //         < VisionConstants.MEGA_TAG_2_DISTANCE_THRESHOLD) {
    //   limelightEstimates[limelight.getId()] =
    // MegatagPoseEstimate.fromLimelight(megatag1Estimate);

    // if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
    //   LimelightHelpers.SetRobotOrientation(limelight.getName(), headingDegrees, 0, 0, 0, 0,
    // 0);
    //   limelightEstimates[limelight.getId()] =
    //       MegatagPoseEstimate.fromLimelight(megatag2Estimate);
    // }
    // else {
    limelightEstimates.set(limelight.getId(), MegatagPoseEstimate.fromLimelight(megatag1Estimate));
    // }
    // }
    // }
    // else {
    //   limelightEstimates[limelight.getId()] = new MegatagPoseEstimate();
    // }
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
  public boolean isValidPoseEstimate(Limelight limelight, PoseEstimate mt1, PoseEstimate mt2) {
    return LimelightHelpers.isValidPoseEstimate(mt1)
        || LimelightHelpers.isValidPoseEstimate(mt2) && isWithinFieldBounds(mt1, mt2);
  }

  /**
   * Checks whether the pose estimate for MT1 or MT2 is within the field
   *
   * @param megaTag1Estimate the MT1 pose estimate to check
   * @param megaTag2Estimate the MT2 pose estimate to check
   */
  private boolean isWithinFieldBounds(
      PoseEstimate megaTag1Estimate, PoseEstimate megaTag2Estimate) {
    return ((megaTag1Estimate.pose.getX() > 0
                && megaTag1Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
            && (megaTag1Estimate.pose.getY() > 0
                && megaTag1Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS))
        || ((megaTag2Estimate.pose.getX() > 0
                && megaTag2Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
            && (megaTag2Estimate.pose.getY() > 0
                && megaTag2Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS));
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

  /**
   * Checks if the ID of the April Tag is within the valid range of 1-22. This is here to check if
   * the IDs the limelight sees are within the range of April Tag IDs on the field. If it randomly
   * sees another April Tag outside of these bounds for whatever reason, the limelight will crash
   * the code, which we don't want.
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @param numberOfAprilTags The number of April Tags detected by the specified Limelight
   * @return True if the ID of the April Tag is within the valid range, false otherwise
   */
  private boolean isValidID(Limelight limelight, int numberOfAprilTags) {
    if (getNumberOfAprilTags(limelight) > 0) {
      return limelightEstimates.get(limelight.getId()).fiducialIds[numberOfAprilTags - 1]
              >= VisionConstants.MIN_APRIL_TAG_ID
          && limelightEstimates.get(limelight.getId()).fiducialIds[numberOfAprilTags - 1]
              <= VisionConstants.MAX_APRIL_TAG_ID;
    }
    return false;
  }

  private boolean isValidMeasurement(Limelight limelight, Pose2d... newPose) {
    return !isSpecificPoseTeleporting(limelight, newPose) && arePosesWithinThreshold(newPose);
  }

  /**
   * Checks if a specific pose in the poses array is teleporting.
   *
   * @param poseIndex The index of the pose in the poses array.
   * @return true if the specific pose is teleporting, false otherwise.
   */
  private boolean isSpecificPoseTeleporting(Limelight limelight, Pose2d... poses) {
    if (limelight.getId() >= 0
        && limelight.getId() < Limelight.values().length
        && poses[limelight.getId()] != null) {
      return isTeleporting(poses[limelight.getId()]);
    }
    return false; // Return false if the index is out of bounds or pose is null.
  }

  private boolean isTeleporting(Pose2d newPose) {
    double distance = odometryPose.getTranslation().getDistance(newPose.getTranslation());
    double rotationDifference =
        Math.abs(odometryPose.getRotation().getDegrees() - newPose.getRotation().getDegrees());

    return distance > VisionConstants.MAX_TRANSLATION_DELTA_METERS
        || rotationDifference > VisionConstants.MAX_ROTATION_DELTA_DEGREES;
  }

  /**
   * Checks if all poses provided are close to each other within a certain threshold.
   *
   * @param thresholdMeters The threshold for translation in meters
   * @param thresholdDegrees The threshold for rotation in degrees
   * @param poses Varargs of Pose2d objects from Limelights
   * @return true if all poses are within the thresholds
   */
  public boolean arePosesWithinThreshold(Pose2d... poses) {
    // Check translations and rotations
    // ARBITRTATATATATRRRY CONSTANTS!
    return GeomUtil.arePosesWithinThreshold(
        VisionConstants.MAX_TRANSLATION_DELTA_METERS,
        VisionConstants.MAX_ROTATION_DELTA_DEGREES,
        poses);
  }

  private boolean isConfident(Limelight limelight) {
    return limelightEstimates.get(limelight.getId()).ambiguity
        >= VisionConstants.MIN_CONFIDENCE_THRESHOLD;
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void checkAndUpdatePose(Limelight limelight) {
    // double last_TX = 0;
    // double last_TY = 0;

    // double current_TX = LimelightHelpers.getTX(limelight.getName());
    // double current_TY = LimelightHelpers.getTY(limelight.getName());

    if (isLimelightConnected(limelight)) {
      if (isValidPoseEstimate(
              limelight, getMegaTag1PoseEstimate(limelight), getMegaTag2PoseEstimate(limelight))
          && isValidMeasurement(limelight, getPoseFromAprilTags(limelight))) {
        // This checks if the limelight reading is new. The reasoning being that if the TX and TY
        // are EXACTLY the same, it hasn't updated yet with a new reading. We are doing it this way,
        // because to get the timestamp of the reading, you need to parse the JSON dump which can be
        // very demanding whereas this only has to get the Network Table entries for TX and TY.
        // if (current_TX != last_TX || current_TY != last_TY
        // idk where to put this \/ thoughts?
        // && isLimelightConnected(limelight)
        // ) {
        // isThreadRunning[limelight.getId()].set(true);
        updatePoseEstimate(limelight);
      }
      // } else {
      //   limelightEstimates[limelight.getId()] = new MegatagPoseEstimate();
      // }
    }

    // } else {
    // // Only stop the thread if it's currently running
    // if (isThreadRunning[limelight.getId()].get()) {
    //   // stop the thread for the specified limelight
    //   stopLimelightThread(limelight);
    // }
    // }

    // last_TX = current_TX;
    // last_TY = current_TY;
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
