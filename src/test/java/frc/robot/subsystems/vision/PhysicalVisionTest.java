import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.PhysicalVision;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PhysicalVisionTest {
  private NetworkTableInstance inst;

  @BeforeEach
  void setUp() {
    // Create a new NetworkTable instance for testing
    inst = NetworkTableInstance.create();
  }

  @AfterEach
  void tearDown() {
    // Destroy the instance after the test
    inst.close();
  }

  @Test
  void testCanSeeAprilTags() {
    // Mock NetworkTables behavior
    var table = inst.getTable("Vision");
    var entry = table.getEntry("aprilTagsVisible");
    entry.setBoolean(true);

    // Call the method under test
    PhysicalVision vision = new PhysicalVision();
    // boolean result = vision.canSeeAprilTags();

    // Assert expected outcome
    assertTrue(result, "Expected canSeeAprilTags() to return true when aprilTagsVisible is true");
  }
}
