package frc.robot.extras.util;

import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.extras.util.ReefLocations;
import frc.robot.extras.util.Obstacle.TeardropObstacle;
import frc.robot.extras.util.Obstacle.VerticalObstacle;
import frc.robot.extras.util.Obstacle.LineObstacle;
import frc.robot.extras.util.Obstacle.PointObstacle;
import frc.robot.extras.util.Obstacle.SnowmanObstacle;
import frc.robot.extras.util.Obstacle.HorizontalObstacle;



/**
 * The Field class encapsulates field dimensions and obstacles for the repulsor field planner.
 * It provides a static list of obstacles that represent reefs, walls, and source boundaries.
 */
public class Field {

    /** The length of the field in meters. */
    public static final double FIELD_LENGTH = 17.6022;

    /** The width of the field in meters. */
    public static final double FIELD_WIDTH = 8.1026;

    /** The X-coordinate used for source obstacles. */
    public static final double SOURCE_X = 1.75;

    /** The Y-coordinate used for source obstacles. */
    public static final double SOURCE_Y = 1.25;

    /**
     * A static list of obstacles placed on the field.
     * This includes reef obstacles, wall obstacles, and source line obstacles.
     */
    public static final List<Obstacle> OBSTACLES = List.of(
        // Reef obstacles
        new TeardropObstacle(ReefLocations.BLUE_REEF, 1, 2, 0.83, 3, 2),
        new TeardropObstacle(ReefLocations.RED_REEF, 1, 2, 0.83, 3, 2),
        // Wall obstacles
        new HorizontalObstacle(0.0, 0.5, 0.5, true),
        new HorizontalObstacle(FIELD_WIDTH, 0.5, 0.5, false),
        new VerticalObstacle(0.0, 0.5, 0.5, true),
        new VerticalObstacle(FIELD_LENGTH, 0.5, 0.5, false),
        // Source obstacles (represented as line obstacles)
        new LineObstacle(
            new Translation2d(0, SOURCE_Y),
            new Translation2d(SOURCE_X, 0),
            0.5,
            0.5),
        new LineObstacle(
            new Translation2d(0, FIELD_WIDTH - SOURCE_Y),
            new Translation2d(SOURCE_X, FIELD_WIDTH),
            0.5,
            0.5),
        new LineObstacle(
            new Translation2d(FIELD_LENGTH, SOURCE_Y),
            new Translation2d(FIELD_LENGTH - SOURCE_X, 0),
            0.5,
            0.5),
        new LineObstacle(
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH - SOURCE_Y),
            new Translation2d(FIELD_LENGTH - SOURCE_X, FIELD_WIDTH),
            0.5,
            0.5)
    );
}
