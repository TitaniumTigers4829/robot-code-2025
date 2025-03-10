// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.extras.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public final class ReefLocations {
  private ReefLocations() {}

  public static final Pose2d[] BLUE_POSES;
  public static final Pose2d[] RED_POSES;

  public static final Translation2d BLUE_REEF;
  public static final Translation2d RED_REEF;

  private static final Translation2d[] BLUE_REEF_WALLS;
  private static final Translation2d[] RED_REEF_WALLS;

  static {
    BLUE_REEF = new Translation2d(4.495, FieldConstants.FIELD_WIDTH_METERS / 2);
    var FIELD_CENTER =
        new Translation2d(
            FieldConstants.FIELD_LENGTH_METERS / 2, FieldConstants.FIELD_WIDTH_METERS / 2);

    var A =
        new Pose2d(
            BLUE_REEF.getX() - 1.45,
            FieldConstants.FIELD_WIDTH_METERS / 2 + .144 - Units.inchesToMeters(2.5),
            Rotation2d.kZero);
    var B =
        new Pose2d(
            BLUE_REEF.getX() - 1.45,
            FieldConstants.FIELD_WIDTH_METERS / 2 - .144 - Units.inchesToMeters(2.5),
            Rotation2d.kZero);

    BLUE_POSES = new Pose2d[12];
    BLUE_POSES[0] = A;
    BLUE_POSES[1] = B;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_POSES[i] =
          new Pose2d(
              A.getTranslation().rotateAround(BLUE_REEF, rotAngle),
              A.getRotation().rotateBy(rotAngle));
      BLUE_POSES[i + 1] =
          new Pose2d(
              B.getTranslation().rotateAround(BLUE_REEF, rotAngle),
              B.getRotation().rotateBy(rotAngle));
    }

    RED_REEF = BLUE_REEF.rotateAround(FIELD_CENTER, Rotation2d.kPi);
    RED_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_POSES[i] =
          new Pose2d(
              BLUE_POSES[i].getTranslation().rotateAround(FIELD_CENTER, Rotation2d.kPi),
              BLUE_POSES[i].getRotation().rotateBy(Rotation2d.kPi));
    }

    var center = new Translation2d(BLUE_REEF.getX() - .85, FieldConstants.FIELD_WIDTH_METERS / 2);
    BLUE_REEF_WALLS = new Translation2d[6];
    RED_REEF_WALLS = new Translation2d[6];
    for (int i = 0; i < 6; i++) {
      var rotAngle = Rotation2d.fromDegrees(60 * i);
      BLUE_REEF_WALLS[i] = center.rotateAround(BLUE_REEF, rotAngle);
      RED_REEF_WALLS[i] = BLUE_REEF_WALLS[i].rotateAround(FIELD_CENTER, Rotation2d.kPi);
    }
  }

  public enum ReefBranch {
    A(0),
    B(1),
    C(2),
    D(3),
    E(4),
    F(5),
    G(6),
    H(7),
    I(8),
    J(9),
    K(10),
    L(11);

    final int id;

    ReefBranch(int id) {
      this.id = id;
    }
  }

  private enum ReefWalls {
    AB(0),
    CD(1),
    EF(2),
    GH(3),
    IJ(4),
    KL(5);

    final int id;

    ReefWalls(int id) {
      this.id = id;
    }
  }

  public static Pose2d getScoringLocation(ReefBranch reefBranch) {
    return (AllianceFlipper.isRed() ? RED_POSES : BLUE_POSES)[reefBranch.id];
  }

  public static Pose2d getSelectedLocation(Translation2d currentPos, boolean left) {
    var walls = AllianceFlipper.isRed() ? RED_REEF_WALLS : BLUE_REEF_WALLS;
    double closestDistance = Double.POSITIVE_INFINITY;
    ReefWalls closestWall = ReefWalls.AB;
    for (var wall : ReefWalls.values()) {
      var dist = walls[wall.id].getDistance(currentPos);
      if (dist < closestDistance) {
        closestWall = wall;
        closestDistance = dist;
      }
    }

    int poseID = closestWall.id * 2 + (left ? 0 : 1);
    return (AllianceFlipper.isRed() ? RED_POSES : BLUE_POSES)[poseID];
  }

  public static void log() {
    Logger.recordOutput("Reef Scoring Locations/Blue", BLUE_POSES);
    Logger.recordOutput("Reef Scoring Locations/Red", RED_POSES);
    Logger.recordOutput("Reef Scoring Locations/Blue Walls", BLUE_REEF_WALLS);
    Logger.recordOutput("Reef Scoring Locations/Red Walls", RED_REEF_WALLS);
  }
}
