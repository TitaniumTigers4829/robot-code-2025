package frc.robot.extras.sim.sim2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.extras.sim.SimArena;
import frc.robot.extras.sim.SimArena.FieldMap;
import frc.robot.extras.sim.SimGamePiece.GamePieceTarget;
import frc.robot.extras.sim.SimGamePiece.GamePieceVariant;
import frc.robot.extras.sim.utils.FieldMirroringUtils;
import frc.robot.extras.util.AllianceFlipper;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;

import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.Geometry;

public class Reefscape { 
  public static final GamePieceVariant ALGAE =
      new GamePieceVariant(
          "Algae", Inches.of(16), Kilograms.of(0.4), Geometry.createCircle(0.176), true, 0.2);
    
/**
 *
 *
 * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
 *
 * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
 *
 * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game environment.
 */
public static class ReefscapeSimArena extends SimArena {
  public static final class ReefscapeFieldObstacleMap extends FieldMap {
      public ReefscapeFieldObstacleMap() {
          super();

          // blue wall
          super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

          // blue coral stations
          super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672, 0));
          super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672, 8.052));

          // red wall
          super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

          // red coral stations
          super.addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548 - 1.672, 0));
          super.addBorderLine(new Translation2d(17.548, 6.782), new Translation2d(17.548 - 1.672, 8.052));

          // upper walls
          super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(17.548 - 1.672, 8.052));

          // lower walls
          super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548 - 1.672, 0));

          // blue reef
          Translation2d[] reefVerticesBlue = new Translation2d[] {
              new Translation2d(3.658, 3.546),
              new Translation2d(3.658, 4.506),
              new Translation2d(4.489, 4.987),
              new Translation2d(5.3213, 4.506),
              new Translation2d(5.3213, 3.546),
              new Translation2d(4.489, 3.065)
          };
          for (int i = 0; i < 6; i++) super.addBorderLine(reefVerticesBlue[i], reefVerticesBlue[(i + 1) % 6]);

          // red reef
          Translation2d[] reefVorticesRed = Arrays.stream(reefVerticesBlue)
                  .map(pointAtBlue ->
                          new Translation2d(FieldMirroringUtils.FIELD_WIDTH - pointAtBlue.getX(), pointAtBlue.getY()))
                  .toArray(Translation2d[]::new);
          for (int i = 0; i < 6; i++) super.addBorderLine(reefVorticesRed[i], reefVorticesRed[(i + 1) % 6]);

          // the pillar in the middle of the field
          super.addRectangularObstacle(0.305, 0.305, new Pose2d(8.774, 4.026, new Rotation2d()));
      }
  }

  public ReefscapeSimArena() {
      super(new ReefscapeFieldObstacleMap());
  }

  @Override
  public void placeGamePiecesOnField() {
      Translation2d[] bluePositions = new Translation2d[] {
          new Translation2d(1.219, 5.855), new Translation2d(1.219, 4.026), new Translation2d(1.219, 2.197),
      };
      for (Translation2d position : bluePositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));

      Translation2d[] redPositions = Arrays.stream(bluePositions)
              .map(bluePosition ->
                      new Translation2d(FieldMirroringUtils.FIELD_WIDTH - bluePosition.getX(), bluePosition.getY()))
              .toArray(Translation2d[]::new);
      for (Translation2d position : redPositions) super.addGamePiece(new ReefscapeCoralAlgaeStack(position));
  }

  @Override
  public void competitionPeriodic() {}

  @Override
  public synchronized List<Pose3d> getGamePiecesByType(String type) {
      List<Pose3d> poses = super.getGamePiecesByType(type);

      // add algae and coral stack
      if (type.equals("Algae")) poses.addAll(ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
      else if (type.equals("Coral")) poses.addAll(ReefscapeCoralAlgaeStack.getStackedCoralPoses());

      return poses;
  }
}}