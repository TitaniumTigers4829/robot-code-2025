package frc.robot.extras.swerve;

import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Before;
import org.junit.Test;

public class RepulsorFieldPlannerTest {

  private RepulsorFieldPlanner planner;

  @Before
  public void setUp() {
    planner = new RepulsorFieldPlanner(); // using parameterized grid resolution
    planner.setGoal(new Translation2d(5, 5));
  }

  /** Test that when the current location equals the goal, the goal force is zero. */
  @Test
  public void testGetGoalForceAtGoal() {
    Translation2d curLocation = new Translation2d(5, 5);
    Translation2d force = planner.getGoalForce(curLocation, new Translation2d(5, 5));
    // Expect zero force if at the goal.
    assertEquals("Goal force should be zero when at the goal", 0, force.getNorm(), 1e-6);
  }

  /**
   * Test that when the robot is away from the goal, the goal force is nonzero and in the correct
   * direction.
   */
  @Test
  public void testGetGoalForceAwayFromGoal() {
    Translation2d curLocation = new Translation2d(3, 3);
    Translation2d goal = new Translation2d(5, 5);
    Translation2d force = planner.getGoalForce(curLocation, goal);
    assertTrue("Goal force should be nonzero", force.getNorm() > 0);

    // Verify the force direction is roughly from curLocation toward goal.
    double expectedAngle = goal.minus(curLocation).getAngle().getRadians();
    double actualAngle = force.getAngle().getRadians();
    // Allow a small tolerance in the angle comparison.
    assertEquals("Force direction should be toward the goal", expectedAngle, actualAngle, 0.1);
  }

  /**
   * Test that the getForce method returns a nonzero net force when obstacles and the goal
   * contribute forces.
   */
  @Test
  public void testGetForceCombinesGoalAndObstacle() {
    Translation2d curLocation = new Translation2d(2, 2);
    Translation2d goal = new Translation2d(5, 5);
    Translation2d totalForce = planner.getForce(curLocation, goal);
    // Since obstacles and the goal force combine, expect a nonzero force.
    assertTrue("Combined force should be nonzero", totalForce.getNorm() > 0);
  }

  /**
   * Test that the arrow grid is generated and cached properly. Calling getArrows twice with the
   * same goal should return the same array.
   */
  @Test
  public void testGetArrowsCaching() {
    Pose2d[] arrowsFirst = planner.getArrows();
    // Call getArrows a second time without changing the goal.
    Pose2d[] arrowsSecond = planner.getArrows();
    // If the goal has not changed, the planner should return the cached grid.
    assertSame("Arrows should be cached and identical", arrowsFirst, arrowsSecond);
  }

  /**
   * Test that the sampleField method produces an intermediate goal that is different from the
   * current location.
   */
  @Test
  public void testSampleFieldProducesIntermediateGoal() {
    Translation2d curLocation = new Translation2d(2, 2);
    double maxSpeed = 3.0; // Maximum speed in m/s (example value)
    double slowdownDistance =
        2.0; // Distance at which the robot begins to slow down (example value)
    RepulsorFieldPlanner.RepulsorSample sample =
        planner.sampleField(curLocation, maxSpeed, slowdownDistance);
    assertNotNull("RepulsorSample should not be null", sample);

    // The intermediate goal should be different from the current location when force is applied.
    assertFalse(
        "Intermediate goal should differ from current location",
        curLocation.equals(sample.intermediateGoal()));
  }
}
