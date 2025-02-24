// TigerHelpers v1.03 (REQUIRES LLOS 2025.0 OR LATER)

package frc.robot.extras.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * TigerHelpers is built on top of LimelightHelpers, providing a set of static methods and classes
 * for interfacing with Limelights, with a dedicated focus on AprilTag pose estimation via
 * Networktables.
 *
 * <p>It includes additional helper methods to simplify common tasks, improve integration with unit
 * testing, and enhance usability while staying up to date with new Limelight features.
 */
public class TigerHelpers {

  private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

  /** Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output. */
  public static class RawFiducial {
    /** The id of the april tag */
    public int id = 0;

    /**
     * The horizontal offset of the target from the Limelight's principle pixel (the images central
     * pixel) in degrees. Positive values mean the target is to the right of the crosshair.
     */
    public double txnc = 0;

    /**
     * The vertical offset of the target from the Limelight's principle pixel (the images central
     * pixel) in degrees. Positive values mean the target is below the crosshair.
     */
    public double tync = 0;

    /**
     * The area of the target in the Limelight's view as a percentage of the total image area. So,
     * if the target takes up 10% of the image, this value will be 0.1.
     */
    public double ta = 0;

    /** The distance from the Limelight to the april tag in meters. */
    public double distToCamera = 0;

    /** The distance from the robot to the april tag in meters. */
    public double distToRobot = 0;

    /**
     * The ambiguity of the april tag. This ranges from 0 to 1, a lower values meaning less
     * ambiguous. The higher this is, the more likely it is that you will see ambiguity flipping.
     * Ambiguity flipping is when there are multiple possible "solutions" for the pose estimate, so
     * if this is higher your estimate will be less reliable. The best way to reduce this it to make
     * the april tag look as trapezoidal as possible from the Limelight's perspective.
     */
    public double ambiguity = 0;

    /**
     * Initializes a RawFidicual with the provided values. This represents data from reading a
     * single april tag.
     *
     * @param id The id of the april tag
     * @param txnc The horizontal offset of the target from the Limelight's principle pixel (the
     *     images central pixel) in degrees.
     * @param tync The vertical offset of the target from the Limelight's principle pixel (the
     *     images central pixel) in degrees.
     * @param ta The area of the target in the Limelight's view as a percentage of the total image
     *     area.
     * @param distToCamera The distance from the Limelight to the april tag in meters.
     * @param distToRobot The distance from the robot to the april tag in meters.
     * @param ambiguity The ambiguity of the april tag. This ranges from 0 to 1, a lower values
     *     meaning less ambiguous.
     */
    public RawFiducial(
        int id,
        double txnc,
        double tync,
        double ta,
        double distToCamera,
        double distToRobot,
        double ambiguity) {
      this.id = id;
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      RawFiducial other = (RawFiducial) obj;
      return id == other.id
          && Double.compare(txnc, other.txnc) == 0
          && Double.compare(tync, other.tync) == 0
          && Double.compare(ta, other.ta) == 0
          && Double.compare(distToCamera, other.distToCamera) == 0
          && Double.compare(distToRobot, other.distToRobot) == 0
          && Double.compare(ambiguity, other.ambiguity) == 0;
    }
  }

  /**
   * Represents a Limelight pose estimate from Limelight's NetworkTables output. This is for any
   * botpose estimate, so MegaTag1 or MegaTag2
   */
  public static class PoseEstimate {
    /** The estimated 2D pose of the robot, including position and rotation. */
    public Pose2d pose;

    /** The timestamp of the pose estimate in seconds since the Limelight booted up. */
    public double timestampSeconds;

    /** The latency of the pose estimate in milliseconds. */
    public double latency;

    /** The number of april tags used to calculate the pose estimate. */
    public int tagCount;

    /** The max distance, in meters, between april tags used to calculate the pose estimate. */
    public double tagSpan;

    /**
     * The average distance, in meters, between the Limelight and the april tags used to calculate
     * the pose estimate.
     */
    public double avgTagDist;

    /** The average area, in square meters, of april tags used to calculate the pose estimate. */
    public double avgTagArea;

    /** An array of RawFiducial used to calculate the pose estimate. */
    public RawFiducial[] rawFiducials;

    /** true if the pose estimate is calculated using MegaTag2, false if using MegaTag1. */
    public boolean isMegaTag2;

    /** Initializes an "empty" PoseEstimate object with default values */
    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    /**
     * Initializes a PoseEstimate object with the provided data.
     *
     * @param pose The estimated 2D pose of the robot, including position and rotation.
     * @param timestampSeconds The timestamp of the pose estimate in seconds since the Limelight
     *     booted up.
     * @param latency The latency of the pose estimate in milliseconds.
     * @param tagCount The number of april tags used to calculate the pose estimate.
     * @param tagSpan The max distance, in meters, between april tags used to calculate the pose
     *     estimate.
     * @param avgTagDist The average distance, in meters, between the Limelight and the april tags
     *     used to calculate the pose estimate.
     * @param avgTagArea The average area, in square meters, of april tags used to calculate the
     *     pose estimate.
     * @param rawFiducials An array of RawFiducial used to calculate the pose estimate.
     * @param isMegaTag2 true if the pose estimate is calculated using MegaTag2, false if using
     *     MegaTag1.
     */
    public PoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        RawFiducial[] rawFiducials,
        boolean isMegaTag2) {

      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.latency = latency;
      this.tagCount = tagCount;
      this.tagSpan = tagSpan;
      this.avgTagDist = avgTagDist;
      this.avgTagArea = avgTagArea;
      this.rawFiducials = rawFiducials;
      this.isMegaTag2 = isMegaTag2;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      PoseEstimate that = (PoseEstimate) obj;
      // We don't compare the timestampSeconds as it isn't relevant for equality and makes
      // unit testing harder
      return Double.compare(that.latency, latency) == 0
          && tagCount == that.tagCount
          && Double.compare(that.tagSpan, tagSpan) == 0
          && Double.compare(that.avgTagDist, avgTagDist) == 0
          && Double.compare(that.avgTagArea, avgTagArea) == 0
          && pose.equals(that.pose)
          && Arrays.equals(rawFiducials, that.rawFiducials);
    }

    @Override
    public String toString() {
      // Convert rawFiducials to a string of just their IDs
      StringBuilder fiducialIds = new StringBuilder("[");
      for (int i = 0; i < rawFiducials.length; i++) {
        fiducialIds.append(rawFiducials[i].id);
        if (i < rawFiducials.length - 1) {
          fiducialIds.append(", ");
        }
      }
      fiducialIds.append("]");

      return "PoseEstimate{"
          + "pose="
          + pose
          + ", timestampSeconds="
          + timestampSeconds
          + ", latency="
          + latency
          + ", tagCount="
          + tagCount
          + ", tagSpan="
          + tagSpan
          + ", avgTagDist="
          + avgTagDist
          + ", avgTagArea="
          + avgTagArea
          + ", rawFiducials="
          + fiducialIds
          + ", isMegaTag2="
          + isMegaTag2
          + '}';
    }
  }

  /**
   * Encapsulates the state of an internal Limelight IMU. This includes gyro and accelerometer data.
   */
  public static class IMUData {
    public double robotYaw;
    public double roll;
    public double pitch;
    public double yaw;
    public double gyroX;
    public double gyroY;
    public double gyroZ;
    public double accelX;
    public double accelY;
    public double accelZ;

    /** Initializes an "empty" IMUData object with default values */
    public IMUData() {
      this.robotYaw = 0;
      this.roll = 0;
      this.pitch = 0;
      this.yaw = 0;
      this.gyroX = 0;
      this.gyroY = 0;
      this.gyroZ = 0;
      this.accelX = 0;
      this.accelY = 0;
      this.accelZ = 0;
    }

    /**
     * Initializes an IMUData object with the provided IMU data array.
     *
     * @param robotYaw The yaw of the robot in degrees
     * @param roll The roll of the robot in degrees
     * @param pitch The pitch of the robot in degrees
     * @param yaw The yaw of the IMU in degrees
     * @param gyroX The x-axis gyro value in degrees per second
     * @param gyroY The y-axis gyro value in degrees per second
     * @param gyroZ The z-axis gyro value in degrees per second TODO: figure out what units the
     *     accelerometer values are in (maybe Gs?)
     * @param accelX The x-axis accelerometer value in meters per second squared
     * @param accelY The y-axis accelerometer value in meters per second squared
     * @param accelZ The z-axis accelerometer value in meters per second squared
     */
    public IMUData(
        double robotYaw,
        double roll,
        double pitch,
        double yaw,
        double gyroX,
        double gyroY,
        double gyroZ,
        double accelX,
        double accelY,
        double accelZ) {
      this.robotYaw = robotYaw;
      this.roll = roll;
      this.pitch = pitch;
      this.yaw = yaw;
      this.gyroX = gyroX;
      this.gyroY = gyroY;
      this.gyroZ = gyroZ;
      this.accelX = accelX;
      this.accelY = accelY;
      this.accelZ = accelZ;
    }
  }

  private static final String sanitizeName(String name) {
    if (name.equals("") || name.equals(null)) {
      return "limelight";
    }
    return name;
  }

  /**
   * Takes a 6-length array of pose data and converts it to a Pose3d object. Array format: [x, y, z,
   * roll, pitch, yaw] where angles are in degrees.
   *
   * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
   * @return Pose3d object representing the pose, or empty Pose3d if invalid data
   */
  public static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  /**
   * Takes a 6-length array of pose data and converts it to a Pose2d object. Uses only x, y, and yaw
   * components, ignoring z, roll, and pitch. Array format: [x, y, z, roll, pitch, yaw] where angles
   * are in degrees.
   *
   * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
   * @return Pose2d object representing the pose, or empty Pose2d if invalid data
   */
  public static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  /**
   * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
   * Translation components are in meters, rotation components are in degrees.
   *
   * @param pose The Pose3d object to convert
   * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
   */
  public static double[] pose3dToArray(Pose3d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = pose.getTranslation().getZ();
    result[3] = Units.radiansToDegrees(pose.getRotation().getX());
    result[4] = Units.radiansToDegrees(pose.getRotation().getY());
    result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
    return result;
  }

  /**
   * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
   * Translation components are in meters, rotation components are in degrees. Note: z, roll, and
   * pitch will be 0 since Pose2d only contains x, y, and yaw.
   *
   * @param pose The Pose2d object to convert
   * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
   */
  public static double[] pose2dToArray(Pose2d pose) {
    double[] result = new double[6];
    result[0] = pose.getTranslation().getX();
    result[1] = pose.getTranslation().getY();
    result[2] = 0;
    result[3] = Units.radiansToDegrees(0);
    result[4] = Units.radiansToDegrees(0);
    result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
    return result;
  }

  private static double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  private static PoseEstimate unpackBotPoseEstimate(
      String limelightName, String entryName, boolean isMegaTag2) {
    DoubleArrayEntry poseEntry =
        TigerHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);

    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) {
      // Handle the case where no data is available
      return new PoseEstimate();
    }

    Pose2d pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);

    // Convert server timestamp from microseconds to seconds and adjust for latency
    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    RawFiducial[] rawFiducials = new RawFiducial[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
      // Don't populate fiducials
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        double txnc = poseArray[baseIndex + 1];
        double tync = poseArray[baseIndex + 2];
        double ta = poseArray[baseIndex + 3];
        double distToCamera = poseArray[baseIndex + 4];
        double distToRobot = poseArray[baseIndex + 5];
        double ambiguity = poseArray[baseIndex + 6];
        rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }

    return new PoseEstimate(
        pose,
        adjustedTimestamp,
        latency,
        tagCount,
        tagSpan,
        tagDist,
        tagArea,
        rawFiducials,
        isMegaTag2);
  }

  /**
   * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Array of RawFiducial objects containing detection details
   */
  public static RawFiducial[] getRawFiducials(String limelightName) {
    NetworkTableEntry entry =
        TigerHelpers.getLimelightNetworkTableEntry(limelightName, "rawfiducials");
    double[] rawFiducialArray = entry.getDoubleArray(new double[0]);
    int valsPerEntry = 7;
    if (rawFiducialArray.length % valsPerEntry != 0) {
      return new RawFiducial[0];
    }

    int numFiducials = rawFiducialArray.length / valsPerEntry;
    RawFiducial[] rawFiducials = new RawFiducial[numFiducials];

    for (int i = 0; i < numFiducials; i++) {
      int baseIndex = i * valsPerEntry;
      int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
      double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
      double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
      double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
      double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
      double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
      double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);

      rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
    }

    return rawFiducials;
  }

  private static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  /**
   * Sets the NetworkTableInstance to use for Limelight communication. Normally it it uses the
   * default instance. This is useful for unit testing when you might need multiple instances.
   */
  public static void setNetworkTableInstance(NetworkTableInstance networkTableInstance) {
    TigerHelpers.networkTableInstance = networkTableInstance;
    clearCaches();
    flushNetworkTable();
  }

  /**
   * Gets the NetworkTableInstance used for Limelight communication.
   *
   * @return The NetworkTableInstance used for Limelight communication
   */
  public static NetworkTableInstance getNetworkTableInstance() {
    return networkTableInstance;
  }

  /**
   * Clears any static caches held in TigerHelpers. This should be called in your test setup (or
   * teardown) to ensure a fresh state between tests.
   */
  public static void clearCaches() {
    doubleArrayEntries.clear();
  }

  /**
   * Flushes the NetworkTables to ensure that any newly updated entry is immediately sent to the
   * network.
   */
  public static void flushNetworkTable() {
    networkTableInstance.flush();
  }

  /**
   * Gets the NetworkTable for a Limelight camera.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @return NetworkTable for the Limelight
   */
  public static NetworkTable getLimelightNetworkTable(String tableName) {
    return networkTableInstance.getTable(sanitizeName(tableName));
  }

  /**
   * Gets a NetworkTableEntry from a Limelight camera.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to get
   * @return NetworkTableEntry for the entry name
   */
  public static NetworkTableEntry getLimelightNetworkTableEntry(
      String tableName, String entryName) {
    return getLimelightNetworkTable(tableName).getEntry(entryName);
  }

  /**
   * Gets a DoubleArrayEntry from a Limelight camera. This is useful for getting raw pose data.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to get
   * @return DoubleArrayEntry for the entry name
   */
  public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
    String key = tableName + "/" + entryName;
    return doubleArrayEntries.computeIfAbsent(
        key,
        k -> {
          NetworkTable table = getLimelightNetworkTable(tableName);
          return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
  }

  /**
   * Gets a NetworkTableEntry from a Limelight camera as a double.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to get
   * @return double value of the entry
   */
  public static double getLimelightNetworkTableDouble(String tableName, String entryName) {
    return getLimelightNetworkTableEntry(tableName, entryName).getDouble(0.0);
  }

  /**
   * Sets a NetworkTableEntry from a Limelight camera as a double.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to set
   * @param val Value to set the entry to
   */
  public static void setLimelightNetworkTableDouble(
      String tableName, String entryName, double val) {
    getLimelightNetworkTableEntry(tableName, entryName).setDouble(val);
  }

  /**
   * Sets a NetworkTableEntry from a Limelight camera as a double array. If you want to the network
   * table array for a {@link PoseEstimate}, use {@link #setBotPoseEstimate(PoseEstimate, String)},
   * if for the {@link RawFiducial}s use {@link #setRawFiducials(RawFiducial[], String)}.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to set
   * @param val Value to set the entry to
   */
  public static void setLimelightNetworkTableDoubleArray(
      String tableName, String entryName, double[] val) {
    getLimelightNetworkTableEntry(tableName, entryName).setDoubleArray(val);
  }

  /**
   * Gets a NetworkTableEntry from a Limelight camera as a double array.
   *
   * @param tableName The name of the Limelight set in the UI ("" for default)
   * @param entryName Name of the entry to get
   * @return double array value of the entry
   */
  public static double[] getLimelightNetworkTableDoubleArray(String tableName, String entryName) {
    return getLimelightNetworkTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  /**
   * Gets if the Limelight have a valid target?
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return True if a valid target is present, false otherwise
   */
  public static boolean getTV(String limelightName) {
    return 1.0 == getLimelightNetworkTableDouble(limelightName, "tv");
  }

  /**
   * Gets the horizontal offset from the crosshair to the target in degrees.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Horizontal offset angle in degrees
   */
  public static double getTX(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "tx");
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Vertical offset angle in degrees
   */
  public static double getTY(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "ty");
  }

  /**
   * Gets the horizontal offset from the principal pixel/point to the target in degrees. This is the
   * most accurate 2d metric if you are using a calibrated camera and you don't need adjustable
   * crosshair functionality.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Horizontal offset angle in degrees
   */
  public static double getTXNC(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "txnc");
  }

  /**
   * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the
   * most accurate 2d metric if you are using a calibrated camera and you don't need adjustable
   * crosshair functionality.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Vertical offset angle in degrees
   */
  public static double getTYNC(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "tync");
  }

  /**
   * Gets the target area as a percentage of the image (0-100%).
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Target area percentage (0-100%)
   */
  public static double getTA(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "ta");
  }

  /**
   * Gets the pipeline's processing latency contribution.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pipeline latency in milliseconds
   */
  public static double getLatencyPipeline(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "tl");
  }

  /**
   * Gets the capture latency.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Capture latency in milliseconds
   */
  public static double getLatencyCapture(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "cl");
  }

  // TODO: deprecate these methods (all the way down to the enum)
  public static double[] getBotPose(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotPose_wpiRed(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "botpose_wpired");
  }

  public static double[] getBotPose_wpiBlue(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "botpose_wpiblue");
  }

  public static double[] getBotPose_TargetSpace(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "botpose_targetspace");
  }

  public static double[] getCameraPose_TargetSpace(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "camerapose_targetspace");
  }

  public static double[] getTargetPose_CameraSpace(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "targetpose_cameraspace");
  }

  public static double[] getTargetPose_RobotSpace(String limelightName) {
    return getLimelightNetworkTableDoubleArray(limelightName, "targetpose_robotspace");
  }

  public static double getFiducialID(String limelightName) {
    return getLimelightNetworkTableDouble(limelightName, "tid");
  }

  public static Pose3d getBotPose3d(String limelightName) {
    double[] poseArray = getLimelightNetworkTableDoubleArray(limelightName, "botpose");
    return toPose3D(poseArray);
  }

  /**
   * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the robot's position and orientation in Red Alliance field
   *     space
   */
  @Deprecated
  public static Pose3d getBotPose3d_wpiRed(String limelightName) {
    double[] poseArray = getLimelightNetworkTableDoubleArray(limelightName, "botpose_wpired");
    return toPose3D(poseArray);
  }

  /**
   * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the robot's position and orientation in Blue Alliance field
   *     space
   */
  @Deprecated
  public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
    double[] poseArray = getLimelightNetworkTableDoubleArray(limelightName, "botpose_wpiblue");
    return toPose3D(poseArray);
  }

  /**
   * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the robot's position and orientation relative to the target
   */
  @Deprecated
  public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
    double[] poseArray = getLimelightNetworkTableDoubleArray(limelightName, "botpose_targetspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the camera's position and orientation relative to the target
   */
  @Deprecated
  public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
    double[] poseArray =
        getLimelightNetworkTableDoubleArray(limelightName, "camerapose_targetspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the target's 3D pose with respect to the camera's coordinate system.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the target's position and orientation relative to the camera
   */
  @Deprecated
  public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
    double[] poseArray =
        getLimelightNetworkTableDoubleArray(limelightName, "targetpose_cameraspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the target's 3D pose with respect to the robot's coordinate system.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the target's position and orientation relative to the robot
   */
  @Deprecated
  public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
    double[] poseArray =
        getLimelightNetworkTableDoubleArray(limelightName, "targetpose_robotspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the camera's 3D pose with respect to the robot's coordinate system.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return Pose3d object representing the camera's position and orientation relative to the robot
   */
  @Deprecated
  public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
    double[] poseArray =
        getLimelightNetworkTableDoubleArray(limelightName, "camerapose_robotspace");
    return toPose3D(poseArray);
  }

  /**
   * This enum represents the different types of botpose data that can be retrieved from the
   * Limelight. Each constant corresponds to a specific NetworkTables entry name and indicates
   * whether it uses the MegaTag2 algorithm or the original MegaTag1 algorithm for pose estimation.
   */
  public enum Botpose {
    /**
     * Botpose data for the blue alliance using the MegaTag1 algorithm. Retrieved from the
     * "botpose_wpiblue" NetworkTables entry.
     */
    BLUE_MEGATAG1("botpose_wpiblue", false),

    /**
     * Botpose data for the blue alliance using the MegaTag2 algorithm. Retrieved from the
     * "botpose_orb_wpiblue" NetworkTables entry.
     */
    BLUE_MEGATAG2("botpose_orb_wpiblue", true),

    /**
     * Botpose data for the red alliance using the MegaTag1 algorithm. Retrieved from the
     * "botpose_wpired" NetworkTables entry.
     */
    RED_MEGATAG1("botpose_wpired", false),

    /**
     * Botpose data for the red alliance using the MegaTag2 algorithm. Retrieved from the
     * "botpose_orb_wpired" NetworkTables entry.
     */
    RED_MEGATAG2("botpose_orb_wpired", true);

    private final String entryName;

    private final boolean isMegaTag2;

    Botpose(String entryName, boolean isMegaTag2) {
      this.entryName = entryName;
      this.isMegaTag2 = isMegaTag2;
    }

    /**
     * Gets the entry name for the botpose type. This is the name of the network table entry that
     * contains the botpose data.
     *
     * @return The network table entry name
     */
    public String getEntryName() {
      return entryName;
    }

    /**
     * Gets if the botpose is calculated using MegaTag2.
     *
     * @return True if the botpose is calculated using MegaTag2, false if using MegaTag1
     */
    public boolean isMegaTag2() {
      return isMegaTag2;
    }
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
    double[] result = getBotPose_wpiBlue(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator
   * (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
    return unpackBotPoseEstimate(limelightName, "botpose_wpiblue", false);
  }

  /**
   * Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator
   * (addVisionMeasurement) in the WPILib Blue alliance coordinate system. Make sure you are calling
   * {setRobotOrientation()} before calling this method.
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
    return unpackBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static Pose2d getBotPose2d_wpiRed(String limelightName) {
    double[] result = getBotPose_wpiRed(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
   * you are on the RED alliance
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
    return unpackBotPoseEstimate(limelightName, "botpose_wpired", false);
  }

  /**
   * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when
   * you are on the RED alliance
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
    return unpackBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
  }

  /**
   * Gets the specified Pose2d for easy use with Odometry vision pose estimator
   * (addVisionMeasurement).
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param botpose the type of botpose to get
   * @return the Pose2d of the robot relative to the origin specified by the botpose
   */
  public static Pose2d getBotPose2d(String limelightName, Botpose botpose) {
    double[] result = getLimelightNetworkTableDoubleArray(limelightName, botpose.getEntryName());
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d with the blue-side origin for use with Odometry vision pose estimator
   * (addVisionMeasurement).
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return the Pose2d of the robot relative to the blue-side origin
   */
  public static Pose2d getBotPose2d(String limelightName) {
    return getBotPose2d(limelightName, Botpose.BLUE_MEGATAG1);
  }

  /**
   * Gets the PoseEstimate for the specified {@link Botpose} type.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param botpose the type of botpose to get
   * @return the PoseEstimate object
   */
  public static PoseEstimate getBotPoseEstimate(String limelightName, Botpose botpose) {
    return unpackBotPoseEstimate(limelightName, botpose.getEntryName(), botpose.isMegaTag2());
  }

  /**
   * Gets the PoseEstimate with the blue-side origin using MegaTag1.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return the PoseEstimate object
   */
  public static PoseEstimate getBotPoseEstimate(String limelightName) {
    return getBotPoseEstimate(limelightName, Botpose.BLUE_MEGATAG1);
  }

  /**
   * Sets the network table entry for the botpose data. This is useful for setting values for unit
   * testing. The {@link PoseEstimate} does not contain values for the z coordinate, roll, and
   * pitch, so these will be set to 0.
   *
   * @param poseEstimate the pose estimate to set
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param botpose the type of botpose to set (e.g., BLUE_MEGATAG1)
   */
  public static void setBotPoseEstimate(
      PoseEstimate poseEstimate, String limelightName, Botpose botpose) {
    // Because network tables don't support 2D arrays, we need to flatten the data into a 1D array
    // Calculates array size: 11 (for PoseEstimate data) + 7 * number of fiducials (for each raw
    // fiducial)
    int fiducialCount = poseEstimate.rawFiducials.length;
    double[] data = new double[11 + 7 * fiducialCount];

    // Populates the PoseEstimate, matching unpackBotPoseEstimate
    data[0] = poseEstimate.pose.getX(); // x
    data[1] = poseEstimate.pose.getY(); // y
    data[2] = 0.0; // z (Pose2d doesn't use this)
    data[3] = 0.0; // roll (not used)
    data[4] = 0.0; // pitch (not used)
    data[5] = poseEstimate.pose.getRotation().getDegrees(); // yaw
    data[6] = poseEstimate.latency; // latency
    data[7] = fiducialCount; // tagCount (must match fiducials length)
    data[8] = poseEstimate.tagSpan; // tagSpan
    data[9] = poseEstimate.avgTagDist; // avgTagDist
    data[10] = poseEstimate.avgTagArea; // avgTagArea

    // Add data for each fiducial
    for (int i = 0; i < fiducialCount; i++) {
      int baseIndex = 11 + (i * 7);
      RawFiducial fid = poseEstimate.rawFiducials[i];
      data[baseIndex] = fid.id; // id (cast to double)
      data[baseIndex + 1] = fid.txnc; // txnc
      data[baseIndex + 2] = fid.tync; // tync
      data[baseIndex + 3] = fid.ta; // ta
      data[baseIndex + 4] = fid.distToCamera; // distToCamera
      data[baseIndex + 5] = fid.distToRobot; // distToRobot
      data[baseIndex + 6] = fid.ambiguity; // ambiguity
    }

    // Write the array to NetworkTables
    setLimelightNetworkTableDoubleArray(limelightName, botpose.getEntryName(), data);
  }

  /**
   * Checks if a PoseEstimate is valid. If this is true, it means the PoseEstimate is has valid data
   * from at least one april tag.
   *
   * @param poseEstimate The PoseEstimate to check
   * @return True if the PoseEstimate is valid, false otherwise
   */
  public static boolean validPoseEstimate(PoseEstimate poseEstimate) {
    return poseEstimate != null
        && poseEstimate.rawFiducials != null
        && poseEstimate.rawFiducials.length != 0;
  }

  /**
   * Sets the network table entry for the blue-side, MegaTag1 botpose data. This is useful for
   * setting values for unit testing. The {@link PoseEstimate} does not contain values for the z
   * coordinate, roll, and pitch, so these will be set to 0.
   *
   * @param poseEstimate the pose estimate to set
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   */
  public static void setBotPoseEstimate(PoseEstimate poseEstimate, String limelightName) {
    setBotPoseEstimate(poseEstimate, limelightName, Botpose.BLUE_MEGATAG1);
  }

  /**
   * Sets the network table entry for the raw fiducials. This is useful for setting values for unit
   * testing.
   *
   * @param rawFiducials the array of raw fiducials to set
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   */
  public static void setRawFiducials(RawFiducial[] rawFiducials, String limelightName) {
    NetworkTableEntry entry = getLimelightNetworkTableEntry(limelightName, "rawfiducials");
    double[] data = new double[rawFiducials.length * 7];

    for (int i = 0; i < rawFiducials.length; i++) {
      int baseIndex = i * 7;
      RawFiducial fid = rawFiducials[i];
      data[baseIndex] = (double) fid.id;
      data[baseIndex + 1] = fid.txnc;
      data[baseIndex + 2] = fid.tync;
      data[baseIndex + 3] = fid.ta;
      data[baseIndex + 4] = fid.distToCamera;
      data[baseIndex + 5] = fid.distToRobot;
      data[baseIndex + 6] = fid.ambiguity;
    }

    entry.setDoubleArray(data);
  }

  /**
   * Gets the current IMU data from NetworkTables. IMU data is formatted as [robotYaw, roll, pitch,
   * yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ]. Returns all zeros if data is invalid or
   * unavailable.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @return IMUData object containing all current IMU data
   */
  public static IMUData getIMUData(String limelightName) {
    double[] imuData = getLimelightNetworkTableDoubleArray(limelightName, "imu");
    if (imuData == null || imuData.length < 10) {
      return new IMUData(); // Returns object with all zeros
    }
    return new IMUData(
        imuData[0],
        imuData[1],
        imuData[2],
        imuData[3],
        imuData[4],
        imuData[5],
        imuData[6],
        imuData[7],
        imuData[8],
        imuData[9]);
  }

  /**
   * Sets the priority tag ID for the Limelight camera. This is used to prioritize a specific tag
   * for detection, which can be useful for tracking a specific target.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param ID ID of the tag to prioritize
   */
  public static void setPriorityTagID(String limelightName, int ID) {
    setLimelightNetworkTableDouble(limelightName, "priorityid", ID);
  }

  /**
   * Sets the crop window for the camera. The crop window in the UI must be completely open. This
   * can be useful for increasing the Limelight's frame rate as the camera will only process the
   * cropped area.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param cropXMin Minimum X value (-1 to 1)
   * @param cropXMax Maximum X value (-1 to 1)
   * @param cropYMin Minimum Y value (-1 to 1)
   * @param cropYMax Maximum Y value (-1 to 1)
   */
  public static void setCropWindow(
      String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double[] entries = new double[4];
    entries[0] = cropXMin;
    entries[1] = cropXMax;
    entries[2] = cropYMin;
    entries[3] = cropYMax;
    setLimelightNetworkTableDoubleArray(limelightName, "crop", entries);
  }

  /**
   * Sets the offset of the april tag tracking point,
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param offsetX Offset in the X direction in meters
   * @param offsetY Offset in the Y direction in meters
   * @param offsetZ Offset in the Z direction in meters
   */
  public static void setFiducial3DOffset(
      String limelightName, double offsetX, double offsetY, double offsetZ) {
    double[] entries = new double[3];
    entries[0] = offsetX;
    entries[1] = offsetY;
    entries[2] = offsetZ;
    setLimelightNetworkTableDoubleArray(limelightName, "fiducial_offset_set", entries);
  }

  /**
   * Sets robot orientation values used by MegaTag2 localization algorithm.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
   * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
   * @param pitch (Unnecessary) Robot pitch in degrees
   * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
   * @param roll (Unnecessary) Robot roll in degrees
   * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
   */
  public static void setRobotOrientation(
      String limelightName,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    double[] entries = new double[6];
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    setLimelightNetworkTableDoubleArray(limelightName, "robot_orientation_set", entries);
    flushNetworkTable();
  }

  /**
   * Enum representing different IMU usage modes for robot orientation or localization. Defines how
   * internal and external IMUs are utilized, including seeding and convergence assistance.
   */
  public enum IMUMode {
    /**
     * Use only an external IMU orientation set by setRobotOrientation() for orientation data. No
     * interaction with the internal IMU.
     */
    EXTERNAL_IMU(0),

    /**
     * Use an external IMU orientation set by setRobotOrientation() as the primary source and seed
     * the internal IMU with its data. This should be used for "zeroing" the internal IMU. The
     * internal IMU is initialized or calibrated using external IMU values.
     */
    EXTERNAL_IMU_SEED_INTERNAL(1),

    /** Use only the internal IMU for orientation data. No reliance on an external IMU. */
    INTERNAL_IMU(2),

    /**
     * Use the internal IMU with MT1-assisted convergence. MegaTag1 provides additional data to
     * improve internal IMU accuracy or stability.
     */
    INTERNAL_MT1_ASSISTED(3),

    /**
     * Use the internal IMU with external IMU-assisted convergence. The external IMU provides
     * supplementary data to enhance internal IMU performance.
     */
    INTERNAL_EXTERNAL_ASSISTED(4);

    private final int modeValue;

    IMUMode(int modeValue) {
      this.modeValue = modeValue;
    }

    /**
     * Returns the integer value associated with this IMU mode.
     *
     * @return The mode value (0 for EXTERNAL_IMU, 1 for EXTERNAL_IMU_SEED_INTERNAL, etc.)
     */
    public int getModeValue() {
      return modeValue;
    }

    /**
     * Returns the IMUMode corresponding to the given integer value.
     *
     * @param value The integer value to convert to an IMUMode
     * @return The matching IMUMode, or null if no match is found
     */
    public static IMUMode fromValue(int value) {
      for (IMUMode mode : IMUMode.values()) {
        if (mode.modeValue == value) {
          return mode;
        }
      }
      return null;
    }
  }

  /**
   * Configures the IMU mode for MegaTag2 Localization
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param imuMode The IMU mode to set, uses the {@link IMUMode} enum.
   */
  public static void setIMUMode(String limelightName, IMUMode imuMode) {
    setLimelightNetworkTableDouble(limelightName, "imumode_set", imuMode.getModeValue());
  }

  /**
   * Sets the 3D point-of-interest offset for the current fiducial pipeline.
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#point-of-interest-tracking
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param x X offset in meters
   * @param y Y offset in meters
   * @param z Z offset in meters
   */
  public static void setFidcuial3DOffset(String limelightName, double x, double y, double z) {
    double[] entries = new double[3];
    entries[0] = x;
    entries[1] = y;
    entries[2] = z;
    setLimelightNetworkTableDoubleArray(limelightName, "fiducial_offset_set", entries);
  }

  /**
   * Overrides the valid AprilTag IDs that will be used for localization. Tags not in this list will
   * be ignored for robot pose estimation.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param validIDs Array of valid AprilTag IDs to track
   */
  public static void setFiducialIDFiltersOverride(String limelightName, int[] validIDs) {
    double[] validIDsDouble = new double[validIDs.length];
    for (int i = 0; i < validIDs.length; i++) {
      validIDsDouble[i] = validIDs[i];
    }
    setLimelightNetworkTableDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
  }

  /**
   * Sets the downscaling factor for AprilTag detection. Increasing downscale can improve
   * performance at the cost of potentially reduced detection range.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param downscale Downscale factor. Valid values: 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set to
   *     0 for pipeline control.
   */
  public static void setFiducialDownscalingOverride(String limelightName, float downscale) {
    int d = 0; // pipeline
    if (downscale == 1.0) {
      d = 1;
    }
    if (downscale == 1.5) {
      d = 2;
    }
    if (downscale == 2) {
      d = 3;
    }
    if (downscale == 3) {
      d = 4;
    }
    if (downscale == 4) {
      d = 5;
    }
    setLimelightNetworkTableDouble(limelightName, "fiducial_downscale_set", d);
  }

  /**
   * Sets the camera pose relative to the robot.
   *
   * @param limelightName The name of the Limelight set in the UI ("" for default)
   * @param forward Forward offset in meters
   * @param side Side offset in meters
   * @param up Up offset in meters
   * @param roll Roll angle in degrees
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   */
  public static void setCameraPoseRobotSpace(
      String limelightName,
      double forward,
      double side,
      double up,
      double roll,
      double pitch,
      double yaw) {
    double[] entries = new double[6];
    entries[0] = forward;
    entries[1] = side;
    entries[2] = up;
    entries[3] = roll;
    entries[4] = pitch;
    entries[5] = yaw;
    setLimelightNetworkTableDoubleArray(limelightName, "camerapose_robotspace_set", entries);
  }
}
