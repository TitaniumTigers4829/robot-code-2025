package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.extras.vision.TigerHelpers;
import frc.robot.extras.vision.TigerHelpers.Botpose;
import frc.robot.extras.vision.TigerHelpers.PoseEstimate;
import frc.robot.extras.vision.TigerHelpers.RawFiducial;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PhysicalVisionTest {
  private NetworkTableInstance networkTableInstance;
  private NetworkTable backLimelightTable;
  private PhysicalVision physicalVision;

  @BeforeEach
  void setUp() {
    // Mocks the NetworkTable instance for testing
    networkTableInstance = NetworkTableInstance.create();
    TigerHelpers.setNTInstance(networkTableInstance);
    backLimelightTable = networkTableInstance.getTable(Limelight.BACK.getName());
    physicalVision = new PhysicalVision();
  }

  @AfterEach
  void tearDown() {
    networkTableInstance.close();
  }

  @Test
  void testCanSeeAprilTags() {
    // We're only going to test using the back limelight here because all of the
    // logic is the same

    // Because tv isn't populated it means the limelight isn't connected
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

    PoseEstimate expectedPoseEstimate =
        new PoseEstimate(
            new Pose2d(1.0, 1.0, new Rotation2d()),
            20.0,
            20.0,
            0,
            5.0,
            5.0,
            0.5,
            new RawFiducial[] {},
            false);

    // Because PoseEstimates are sent over the network tables as just an array of
    // doubles, we have to manually set the values for the test
    TigerHelpers.setBotPoseEstimate(expectedPoseEstimate, Limelight.BACK.getName());
    assertEquals(expectedPoseEstimate, physicalVision.getMegaTag1PoseEstimate(Limelight.BACK));
  }

  @Test
  void testGetMegaTag2PoseEstimate() {
    // This is just a duplicate of the previous test, but for MT2
    assertEquals(new PoseEstimate(), physicalVision.getMegaTag2PoseEstimate(Limelight.BACK));

    PoseEstimate expectedPoseEstimate =
        new PoseEstimate(
            new Pose2d(1.0, 1.0, new Rotation2d()),
            20.0,
            20.0,
            0,
            5.0,
            5.0,
            0.5,
            new RawFiducial[] {},
            true);

    TigerHelpers.setBotPoseEstimate(
        expectedPoseEstimate, Limelight.BACK.getName(), Botpose.BLUE_MEGATAG2);
    assertEquals(expectedPoseEstimate, physicalVision.getMegaTag2PoseEstimate(Limelight.BACK));
  }

  @Test
  void testEnabledPoseUpdate() {
    // TODO: Implement this test once we finalize MT2 logic
  }

  @Test
  void testLimelightEstimatesGetters() {
    // First we test that the getters return their default values before we set
    // limelightEstimates
    assertEquals(new Pose2d(), physicalVision.getPoseFromAprilTags(Limelight.BACK));
    assertEquals(0, physicalVision.getNumberOfAprilTags(Limelight.BACK));
    // Because of how doubles work, we can't compare them directly, so we have to
    // also have a delta/range to check if it's "close enough"
    assertEquals(0.0, physicalVision.getTimestampSeconds(Limelight.BACK));
    assertEquals(0.0, physicalVision.getLatencySeconds(Limelight.BACK));
    assertEquals(Double.MAX_VALUE, physicalVision.getLimelightAprilTagDistance(Limelight.BACK));
    assertEquals(0.0, physicalVision.getAmbiguity(Limelight.BACK));

    PoseEstimate expectedPoseEstimate =
        new PoseEstimate(
            new Pose2d(1.0, 1.0, new Rotation2d()),
            20.0,
            20.0,
            0,
            5.0,
            5.0,
            0.5,
            // new RawFiducial[] {new RawFiducial(1, 0, 0, 1, 1, 1, 0.2)},
            new RawFiducial[] {},
            false);

    TigerHelpers.setBotPoseEstimate(expectedPoseEstimate, Limelight.BACK.getName());

    // // For simplictity, we're just going to use the disabledPoseUpdate method to set
    // // the limelightEstimates as it doesn't have any logic in it like
    // // enabledPoseUpdate
    physicalVision.disabledPoseUpdate(Limelight.BACK);
    // assertEquals(expectedPoseEstimate.pose, physicalVision.getPoseFromAprilTags(Limelight.BACK));
    // assertEquals(
    //     expectedPoseEstimate.tagCount, physicalVision.getNumberOfAprilTags(Limelight.BACK));
    // assertEquals(
    //     expectedPoseEstimate.latency, physicalVision.getLatencySeconds(Limelight.BACK), 0.0);
    // assertEquals(
    //     expectedPoseEstimate.avgTagDist,
    //     physicalVision.getLimelightAprilTagDistance(Limelight.BACK),
    //     0.0);
    // assertEquals(
    //     expectedPoseEstimate.rawFiducials[0].ambiguity,
    //     physicalVision.getAmbiguity(Limelight.BACK),
    //     0.0);
  }

  @Test
  void testIsValidMeasurement() {
    // // If the pose hasn't been updated, it should return false
    // assertFalse(physicalVision.isValidMeasurement(Limelight.BACK));

    // // If the pose is not empty, within the field, is confident, and not
    // // teleporting, then it should be true
    // PoseEstimate expectedPoseEstimate =
    //     new PoseEstimate(
    //         new Pose2d(
    //             FieldConstants.FIELD_LENGTH_METERS / 2.0,
    //             FieldConstants.FIELD_WIDTH_METERS / 2.0,
    //             new Rotation2d()),
    //         20.0,
    //         20.0,
    //         1,
    //         5.0,
    //         5.0,
    //         0.5,
    //         new RawFiducial[] {
    //           new RawFiducial(1, 0, 0, 1, 1, 1, VisionConstants.MAX_AMBIGUITY_THRESHOLD * 0.9)
    //         },
    //         false);

    // backLimelightTable
    //     .getEntry("botpose_wpiblue")
    //     .setDoubleArray(
    //         new double[] {
    //           expectedPoseEstimate.pose.getX(),
    //           expectedPoseEstimate.pose.getY(),
    //           0.0,
    //           0.0,
    //           0.0,
    //           expectedPoseEstimate.pose.getRotation().getDegrees(),
    //           expectedPoseEstimate.latency,
    //           expectedPoseEstimate.tagCount,
    //           expectedPoseEstimate.tagSpan,
    //           expectedPoseEstimate.avgTagDist,
    //           expectedPoseEstimate.avgTagArea
    //         });

    // // Update the limeightEstimates
    // physicalVision.disabledPoseUpdate(Limelight.BACK);

    // assertTrue(physicalVision.isValidMeasurement(Limelight.BACK));

    // // If we make the make the ambiguity high, it should return false
    // expectedPoseEstimate =
    //     new PoseEstimate(
    //         new Pose2d(
    //             FieldConstants.FIELD_LENGTH_METERS * 0.5,
    //             FieldConstants.FIELD_WIDTH_METERS * 0.5,
    //             new Rotation2d()),
    //         20.0,
    //         20.0,
    //         1,
    //         5.0,
    //         5.0,
    //         0.5,
    //         new RawFiducial[] {
    //           new RawFiducial(1, 0, 0, 1, 1, 1, VisionConstants.MAX_AMBIGUITY_THRESHOLD * 1.1)
    //         },
    //         false);
    // physicalVision.disabledPoseUpdate(Limelight.BACK);
    // assertFalse(physicalVision.isValidMeasurement(Limelight.BACK));

    // // If we make it start teleporting, it should return false
    // expectedPoseEstimate =
    //     new PoseEstimate(
    //         new Pose2d(
    //             FieldConstants.FIELD_LENGTH_METERS * 0.75,
    //             FieldConstants.FIELD_WIDTH_METERS * 0.75,
    //             new Rotation2d()),
    //         20.0,
    //         20.0,
    //         1,
    //         5.0,
    //         5.0,
    //         0.5,
    //         new RawFiducial[] {
    //           new RawFiducial(1, 0, 0, 1, 1, 1, VisionConstants.MAX_AMBIGUITY_THRESHOLD * 0.9)
    //         },
    //         false);
    // physicalVision.disabledPoseUpdate(Limelight.BACK);
    // assertFalse(physicalVision.isValidMeasurement(Limelight.BACK));
  }
}
