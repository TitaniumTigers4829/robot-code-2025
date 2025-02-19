import static org.junit.Assert.assertFalse;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.PhysicalVision;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PhysicalVisionTest {
  private NetworkTableInstance networkTableInstance;
  private PhysicalVision physicalVision;

  @BeforeEach
  void setUp() {
    // Mocks the NetworkTable instance for testing
    networkTableInstance = NetworkTableInstance.getDefault();
    physicalVision = new PhysicalVision();
  }

  @AfterEach
  void tearDown() {
    // Destroy the instance after the test
    networkTableInstance.close();
  }

  @Test
  void testCanSeeAprilTags() {
    // We're only going to test one limelight here because all of the logic is the same
    NetworkTable table = networkTableInstance.getTable(Limelight.BACK.getName());

    // Because tv is 0, it means the limelight doesn't see any targets, so it should return false
    table.getEntry("tv").setDouble(0.0);
    assertFalse(physicalVision.canSeeAprilTags(Limelight.BACK));

    // Because tx is greater than the accurate FOV, it should return false
    table.getEntry("tv").setDouble(1.0);
    table.getEntry("tx").setDouble(Limelight.BACK.getAccurateFOV() + 1);
    assertFalse(physicalVision.canSeeAprilTags(Limelight.BACK));

    // If tv is 1 and tx is less than the accurate FOV, it should return true, regardless
    // of tx being negative, positive, or zero
    table.getEntry("tx").setDouble(-(Limelight.BACK.getAccurateFOV() - 1));
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
    table.getEntry("tx").setDouble((Limelight.BACK.getAccurateFOV() - 1));
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
    table.getEntry("tx").setDouble(0.0);
    assertTrue(physicalVision.canSeeAprilTags(Limelight.BACK));
  }
}
