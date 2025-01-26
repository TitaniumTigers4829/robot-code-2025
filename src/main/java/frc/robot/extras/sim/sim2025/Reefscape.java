package frc.robot.extras.sim.sim2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.sim.SimArena;
import frc.robot.extras.sim.SimArena.FieldMap;
import frc.robot.extras.sim.SimGamePiece.GamePieceTarget;
import frc.robot.extras.sim.SimGamePiece.GamePieceVariant;
import frc.robot.extras.util.utils.FieldMirroringUtils;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.dyn4j.geometry.Geometry;

public class Reefscape {
  private static List<GamePieceTarget> createCoralTargets() {
    List<Translation3d> reefPositions =
        Arrays.asList(
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_ONE.getX(),
                Constants.FieldConstants.BLUE_REEF_ONE.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_TWO.getX(),
                Constants.FieldConstants.BLUE_REEF_TWO.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_THREE.getX(),
                Constants.FieldConstants.BLUE_REEF_THREE.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_FOUR.getX(),
                Constants.FieldConstants.BLUE_REEF_FOUR.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_FIVE.getX(),
                Constants.FieldConstants.BLUE_REEF_FIVE.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_SIX.getX(),
                Constants.FieldConstants.BLUE_REEF_SIX.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_SEVEN.getX(),
                Constants.FieldConstants.BLUE_REEF_SEVEN.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_EIGHT.getX(),
                Constants.FieldConstants.BLUE_REEF_EIGHT.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_NINE.getX(),
                Constants.FieldConstants.BLUE_REEF_NINE.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_TEN.getX(),
                Constants.FieldConstants.BLUE_REEF_TEN.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_ELEVEN.getX(),
                Constants.FieldConstants.BLUE_REEF_ELEVEN.getY(),
                0.0),
            new Translation3d(
                Constants.FieldConstants.BLUE_REEF_TWELEVE.getX(),
                Constants.FieldConstants.BLUE_REEF_TWELEVE.getY(),
                0.0));

    List<Double> zLevels =
        Arrays.asList(
            Constants.FieldConstants.REEF_LEVEL_ONE_Z,
            Constants.FieldConstants.REEF_LEVEL_TWO_Z,
            Constants.FieldConstants.REEF_LEVEL_THREE_Z,
            Constants.FieldConstants.REEF_LEVEL_FOUR_Z);

    return reefPositions.stream()
        .flatMap(
            position ->
                zLevels.stream()
                    .map(
                        z ->
                            new GamePieceTarget(
                                new Translation3d(position.getX(), position.getY(), z),
                                new Translation3d(position.getX(), position.getY(), z))))
        .collect(Collectors.toList());
  }

  private static List<GamePieceTarget> createAlgaeTargets() {
    return Arrays.asList(
        new GamePieceTarget(
            new Translation3d(
                FieldConstants.BLUE_PROCESSOR.getX(),
                FieldConstants.BLUE_PROCESSOR.getY(),
                FieldConstants.PROCESSOR_Z),
            new Translation3d(
                FieldConstants.RED_PROCESSOR.getX(),
                FieldConstants.RED_PROCESSOR.getY(),
                FieldConstants.PROCESSOR_Z)));
  }

  public static final List<GamePieceTarget> ALGAE_TARGETS = createAlgaeTargets();

  public static final List<GamePieceTarget> CORAL_TARGETS = createCoralTargets();

  public static final GamePieceVariant ALGAE =
      new GamePieceVariant(
          "Algae",
          Units.inchesToMeters(16),
          0.4,
          Geometry.createCircle(0.176),
          ALGAE_TARGETS,
          true,
          0.2);
  public static final GamePieceVariant CORAL =
      new GamePieceVariant(
          "Coral",
          Units.inchesToMeters(16),
          0.4,
          Geometry.createCircle(0.176),
          CORAL_TARGETS,
          true,
          0.2);

  /**
   *
   *
   * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
   *
   * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
   *
   * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game
   * environment.
   */
  public static class ReefscapeSimArena extends SimArena {
    public static final class ReefscapeFieldObstacleMap extends FieldMap {
      public ReefscapeFieldObstacleMap() {
        super();

        // blue wall
        addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

        // blue coral stations
        addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
        addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

        // red wall
        addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

        // red coral stations
        addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548 - 1.672, 0));
        addBorderLine(new Translation2d(17.548, 6.782), new Translation2d(17.548 - 1.672, 8.052));

        // upper walls
        addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(17.548 - 1.672, 8.052));

        // lower walls
        addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548 - 1.672, 0));

        // blue reef
        Translation2d[] reefVerticesBlue =
            new Translation2d[] {
              new Translation2d(3.658, 3.546),
              new Translation2d(3.658, 4.506),
              new Translation2d(4.489, 4.987),
              new Translation2d(5.3213, 4.506),
              new Translation2d(5.3213, 3.546),
              new Translation2d(4.489, 3.065)
            };
        for (int i = 0; i < 6; i++)
          super.addBorderLine(reefVerticesBlue[i], reefVerticesBlue[(i + 1) % 6]);

        // red reef
        Translation2d[] reefVorticesRed =
            Arrays.stream(reefVerticesBlue)
                .map(
                    pointAtBlue ->
                        new Translation2d(
                            FieldMirroringUtils.FIELD_WIDTH - pointAtBlue.getX(),
                            pointAtBlue.getY()))
                .toArray(Translation2d[]::new);
        for (int i = 0; i < 6; i++)
          super.addBorderLine(reefVorticesRed[i], reefVorticesRed[(i + 1) % 6]);

        // the pillar in the middle of the field
        addRectangularObstacle(0.305, 0.305, new Pose2d(8.774, 4.026, new Rotation2d()));
      }
    }

    public ReefscapeSimArena(Time period, int simulationSubTick) {
      super(new ReefscapeFieldObstacleMap(), period.in(Seconds), simulationSubTick);
    }

    @Override
    public void competitionPeriodic() {}

    @Override
    protected void placeGamePiecesOnField() {
      Translation2d[] bluePositions =
          new Translation2d[] {
            new Translation2d(1.219, 5.855),
            new Translation2d(1.219, 4.026),
            new Translation2d(1.219, 2.197),
          };

      Translation2d[] redPositions =
          Arrays.stream(bluePositions)
              .map(
                  bluePosition ->
                      new Translation2d(
                          FieldMirroringUtils.FIELD_WIDTH - bluePosition.getX(),
                          bluePosition.getY()))
              .toArray(Translation2d[]::new);
    }
  }
}
