package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.util.GeomUtil;
import frc.robot.extras.vision.LimelightHelpers;
import frc.robot.extras.vision.LimelightHelpers.PoseEstimate;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is the implementation of the VisionInterface for the physical robot. It uses the
 * ThreadManager to make threads to run the code for processing the vision data from the limelights
 * asynchonously.
 *
 * @author Jack
 * @author Ishan
 */
public class PhysicalVision implements VisionInterface {

  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;

  private final ConcurrentHashMap<Limelight, AtomicReference<VisionInputs>> limelightThreads =
      new ConcurrentHashMap<>();
  private final AtomicReference<VisionInputs> latestInputs =
      new AtomicReference<>(new VisionInputs());
  private final ThreadManager threadManager = new ThreadManager(Limelight.values().length);

  // private final RawFiducial rawFiducial = new RawFiducial();

  /**
   * The pose estimates from the limelights in the following order (BACK, FRONT_LEFT, FRONT_RIGHT)
   */
  private MegatagPoseEstimate[] limelightEstimates =
      new MegatagPoseEstimate[] {
        new MegatagPoseEstimate(), new MegatagPoseEstimate(), new MegatagPoseEstimate()
      };

  public PhysicalVision() {
    for (Limelight limelight : Limelight.values()) {
      limelightThreads.put(limelight, new AtomicReference<>(latestInputs.get()));

      // Start a vision input task for each Limelight
      threadManager.startVisionInputTask(
          limelight.getName(),
          latestInputs.get(),
          () -> checkAndUpdatePose(limelight, latestInputs.get()));
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // Combine inputs into the main inputs object
    synchronized (inputs) {
      for (Limelight limelight : Limelight.values()) {
        inputs.isLimelightConnected[limelight.getId()] = isLimelightConnected(limelight);
        inputs.limelightLatencies[limelight.getId()] = getLatencySeconds(limelight);
        inputs.limelightTargets[limelight.getId()] = getNumberOfAprilTags(limelight);
        inputs.limelightSeesAprilTags[limelight.getId()] = canSeeAprilTags(limelight);
        inputs.limelightMegatagPoses[limelight.getId()] = limelightEstimates[limelight.getId()];
        inputs.limelightAprilTagDistances[limelight.getId()] =
            getLimelightAprilTagDistance(limelight);
        inputs.limelightCalculatedPoses[limelight.getId()] = getPoseFromAprilTags(limelight);
        inputs.limelightTimestamps[limelight.getId()] = getTimeStampSeconds(limelight);
        inputs.limelightLastSeenPose = getLastSeenPose();

        latestInputs.set(inputs);
        limelightThreads.get(limelight).set(latestInputs.get());
      }
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
    return limelightEstimates[limelight.getId()].fieldToCamera;
  }

  @Override
  public int getNumberOfAprilTags(Limelight limelight) {
    return limelightEstimates[limelight.getId()].tagCount;
  }

  @Override
  public double getTimeStampSeconds(Limelight limelight) {
    return limelightEstimates[limelight.getId()].timestampSeconds / 1000.0;
  }

  @Override
  public double getLatencySeconds(Limelight limelight) {
    return (limelightEstimates[limelight.getId()].latency) / 1000.0;
  }

  @Override
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  @Override
  public double getLimelightAprilTagDistance(Limelight limelight) {
    if (canSeeAprilTags(limelight)) {
      return limelightEstimates[limelight.getId()].avgTagDist;
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  @Override
  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
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

    if (canSeeAprilTags(limelight)
        && isValidPoseEstimate(limelight, megatag1Estimate, megatag2Estimate)) {
      if (isLargeDiscrepancyBetweenMegaTag1And2(limelight, megatag1Estimate, megatag2Estimate)
          && getLimelightAprilTagDistance(limelight)
              < VisionConstants.MEGA_TAG_2_DISTANCE_THRESHOLD) {
        limelightEstimates[limelight.getId()] = MegatagPoseEstimate.fromLimelight(megatag1Estimate);
      } else if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(limelight.getName(), headingDegrees, 0, 0, 0, 0, 0);
        limelightEstimates[limelight.getId()] = MegatagPoseEstimate.fromLimelight(megatag2Estimate);
      } else {
        limelightEstimates[limelight.getId()] = MegatagPoseEstimate.fromLimelight(megatag1Estimate);
      }
    }
    limelightEstimates[limelight.getId()] = new MegatagPoseEstimate();
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void disabledPoseUpdate(Limelight limelight) {
    limelightEstimates[limelight.getId()] =
        MegatagPoseEstimate.fromLimelight(getMegaTag1PoseEstimate(limelight));
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void updatePoseEstimate(Limelight limelight, VisionInputs inputs) {
    synchronized (inputs) {
      if (DriverStation.isEnabled()) {
        enabledPoseUpdate(limelight);
      } else {
        disabledPoseUpdate(limelight);
      }
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
    return !GeomUtil.isTranslationWithinThreshold(
            mt1.pose.getTranslation(),
            mt2.pose.getTranslation(),
            VisionConstants.MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD)
        || !GeomUtil.isRotationWithinThreshold(
            mt1.pose.getRotation().getDegrees(),
            mt2.pose.getRotation().getDegrees(),
            VisionConstants.MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD);
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
        && LimelightHelpers.isValidPoseEstimate(mt2)
        && isWithinFieldBounds(mt1, mt2);
  }

  /**
   * Checks whether the pose estimate for MT1 and MT2 is within the field
   *
   * @param megaTag1Estimate the MT1 pose estimate to check
   * @param megaTag2Estimate the MT2 pose estimate to check
   */
  private boolean isWithinFieldBounds(
      PoseEstimate megaTag1Estimate, PoseEstimate megaTag2Estimate) {
    return (megaTag1Estimate.pose.getX() > 0
            && megaTag1Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag1Estimate.pose.getY() > 0
            && megaTag1Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag2Estimate.pose.getX() > 0
            && megaTag2Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag2Estimate.pose.getY() > 0
            && megaTag2Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS);
  }

  /**
   * Checks if the ID of the April Tag is within the valid range of 1-22. 
   * This is here to check if the IDs the limelight sees are within the range of April Tag IDs on the field.
   *  If it randomly sees another April Tag outside of these bounds for whatever reason, 
   * the limelight will crash the code, which we don't want. 
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @param numberOfAprilTags The number of April Tags detected by the specified Limelight
   * @return True if the ID of the April Tag is within the valid range, false otherwise
   */
  private boolean isValidID(Limelight limelight, int numberOfAprilTags) {
    return limelightEstimates[limelight.getId()].fiducialIds[numberOfAprilTags - 1]
            > VisionConstants.MIN_APRIL_TAG_ID
        && limelightEstimates[limelight.getId()].fiducialIds[numberOfAprilTags - 1]
            < VisionConstants.MAX_APRIL_TAG_ID;
  }

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(Limelight limelight) {
    NetworkTable limelightTable = LimelightHelpers.getLimelightNTTable(limelight.getName());
    return limelightTable.containsKey("tv");
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   */
  public void checkAndUpdatePose(Limelight limelight, VisionInputs inputs) {
    double last_TX = 0;
    double last_TY = 0;

    // Syncronization block to ensure thread safety during the critical section where pose
    // information is read and compared.
    // This helps prevent race conditions, where one limelight may be updating an object that
    // another limelight is reading.
    // A race condition could cause unpredictable things to happen. Such as causing a limelight to
    // be unable to reference an
    // object, as its reference was modified earlier.
    synchronized (this) {
      try {
        double current_TX = LimelightHelpers.getTX(limelight.getName());
        double current_TY = LimelightHelpers.getTY(limelight.getName());

        // This checks if the limelight reading is new. The reasoning being that if the TX and TY
        // are EXACTLY the same, it hasn't updated yet with a new reading. We are doing it this way,
        // because to get the timestamp of the reading, you need to parse the JSON dump which can be
        // very demanding whereas this only has to get the Network Table entries for TX and TY.
        if (current_TX != last_TX || current_TY != last_TY) {

          updatePoseEstimate(limelight, inputs);
          latestInputs.set(inputs);

          limelightThreads.computeIfPresent(limelight, (key, value) -> latestInputs);
          // Handle threading for Limelight (start or stop threads if needed)
          // Check if this Limelight thread exists in limelightThreads
          if (limelightThreads.get(limelight) != null) {
            // Update thread inputs or restart the thread if needed
            limelightThreads.get(limelight).set(latestInputs.get());
          }

          // This is to keep track of the last valid pose calculated by the limelights
          // it is used when the driver resets the robot odometry to the limelight calculated
          // position
          if (canSeeAprilTags(limelight)) {
            lastSeenPose = getMegaTag1PoseEstimate(limelight).pose;
          }
        } else {
          // Retrieve the AtomicReference for the given limelight number
          AtomicReference<VisionInputs> isThreadRunning =
              limelightThreads.getOrDefault(limelight, latestInputs);
          // Only stop the thread if it's currently running
          if (isThreadRunning.get() != null) {
            // stop the thread for the specified limelight
            stopLimelightThread(limelight);
          }
        }
        last_TX = current_TX;
        last_TY = current_TY;
      } catch (Exception e) {
        System.err.println(
            "Error communicating with the: " + limelight.getName() + ": " + e.getMessage());
      }
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
