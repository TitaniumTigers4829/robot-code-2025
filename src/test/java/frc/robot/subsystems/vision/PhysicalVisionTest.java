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

  @BeforeEach
  void setUp() {
    // Mocks the NetworkTable instance for testing
    networkTableInstance = NetworkTableInstance.getDefault();
  }

  @AfterEach
  void tearDown() {
    // Destroy the instance after the test
    networkTableInstance.close();
  }

  @Test
  void testCanSeeAprilTags() {
    NetworkTable table = networkTableInstance.getTable(Limelight.BACK.getName());
    table.getEntry("tv").setDouble(1.0);
    table.getEntry("tx").setDouble(0.0);

    PhysicalVision vision = new PhysicalVision();

    assertTrue(vision.canSeeAprilTags(Limelight.BACK));
  }
}
