package frc.robot.subsystems.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.extras.vision.TigerHelpers.PoseEstimate;
import frc.robot.extras.vision.TigerHelpers.RawFiducial;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class PhysicalVisionTest {
  private NetworkTableInstance networkTableInstance;
  private NetworkTable backLimelightTable;
  private PhysicalVision physicalVision;

  @Before
  void setUp() {
    // Mocks the NetworkTable instance for testing
    networkTableInstance = NetworkTableInstance.getDefault();
    backLimelightTable = networkTableInstance.getTable(Limelight.BACK.getName());
    physicalVision = new PhysicalVision();
  }

  @After
  void tearDown() {
    // Destroy the instance after the test
    networkTableInstance.close();
  }

  @Test
  void testCanSeeAprilTags() {
    // We're only going to test using the back limelight here because all of the
    // logic is the same

    // Because tv is 0, it means the limelight doesn't see any targets, so it should
    // return false
    backLimelightTable.getEntry("tv").setDouble(0.0);
    assertFalse(physicalVision.canSeeAprilTags(Limelight.BACK));

    // Because tx is greater than the accurate FOV, it should return false
    backLimelightTable.getEntry("tv").setDouble(1.0);
    backLimelightTable.getEntry("tx").setDouble(Limelight.BACK.getAccurateFOV() + 1);
    assertFalse(physicalVision.canSeeAprilTags(Limelight.BACK));

    // If tv is 1 and tx is less than the accurate FOV, it should return true,
    // regardless of tx being negative, positive, or zero
    backLimelightTable.getEntry("tx").setDouble(-(Limelight.BACK.getAccurateFOV() - 1));
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
    backLimelightTable.getEntry("tx").setDouble((Limelight.BACK.getAccurateFOV() - 1));
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
    backLimelightTable.getEntry("tx").setDouble(0.0);
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
  }

  @Test
  void testGetMegaTag1PoseEstimate() {
    // When the limelight doesn't see any targets, it should return an empty
    // PoseEstimate
    assertEquals(new PoseEstimate(), physicalVision.getMegaTag1PoseEstimate(Limelight.BACK));

    PoseEstimate expectedPoseEstimate = new PoseEstimate(
        new Pose2d(1.0, 1.0, new Rotation2d()),
        20.0,
        20.0,
        1,
        5.0,
        5.0,
        0.5,
        new RawFiducial[] {},
        false);

    // Because PoseEstimates are sent over the network tables as just an array of
    // doubles, we have to manually set the values for the test
    // The array is: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan,
    // avgTagDist, avgTagArea] units in meters, degrees, and milliseconds
    backLimelightTable
        .getEntry("botpose_wpiblue")
        // These values don't matter, we're just testing if the method can get the
        // values, also the double array has values to make a Pose3d with x, y, z, roll,
        // pitch, yaw, but PoseEstimate only uses a Pose2d
        .setDoubleArray(
            new double[] {
                expectedPoseEstimate.pose.getX(),
                expectedPoseEstimate.pose.getY(),
                0.0,
                0.0,
                0.0,
                expectedPoseEstimate.pose.getRotation().getDegrees(),
                expectedPoseEstimate.latency,
                expectedPoseEstimate.tagCount,
                expectedPoseEstimate.tagSpan,
                expectedPoseEstimate.avgTagDist,
                expectedPoseEstimate.avgTagArea
            });
  }

  @Test
  void testGetMegaTag2PoseEstimate() {
    // This is just a duplicate of the previous test, but for MT2
    assertEquals(new PoseEstimate(), physicalVision.getMegaTag2PoseEstimate(Limelight.BACK));

    PoseEstimate expectedPoseEstimate = new PoseEstimate(
        new Pose2d(1.0, 1.0, new Rotation2d()),
        20.0,
        20.0,
        1,
        5.0,
        5.0,
        0.5,
        new RawFiducial[] {},
        true);

    backLimelightTable
        .getEntry("botpose_orb_wpiblue")
        .setDoubleArray(
            new double[] {
                expectedPoseEstimate.pose.getX(),
                expectedPoseEstimate.pose.getY(),
                0.0,
                0.0,
                0.0,
                expectedPoseEstimate.pose.getRotation().getDegrees(),
                expectedPoseEstimate.latency,
                expectedPoseEstimate.tagCount,
                expectedPoseEstimate.tagSpan,
                expectedPoseEstimate.avgTagDist,
                expectedPoseEstimate.avgTagArea
            });
  }

  
}
