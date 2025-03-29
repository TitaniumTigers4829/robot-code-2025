// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.extras.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.extras.util.Field;
import frc.robot.extras.util.Obstacle;
import frc.robot.extras.util.ReefLocations;
import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {

  private Translation2d goal = new Translation2d(1, 1);

  private static final int ARROWS_X = 40;
  private static final int ARROWS_Y = 20;

  private Translation2d lastGoal;
  private Pose2d[] arrowList;

  // A grid of arrows drawn in AScope
  public Pose2d[] getArrows() {
    if (goal.equals(lastGoal)) {
      return arrowList;
    }
    var list = new ArrayList<Pose2d>();
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(x * Field.FIELD_LENGTH / ARROWS_X, y * Field.FIELD_WIDTH / ARROWS_Y);
        var force = getObstacleForce(translation, goal);
        if (force.getNorm() > 1e-6) {
          var rotation = force.getAngle();

          list.add(new Pose2d(translation, rotation));
        }
      }
    }
    lastGoal = goal;
    arrowList = list.toArray(new Pose2d[0]);
    return arrowList;
  }

  Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Translation2d();
    }
    Rotation2d direction = displacement.getAngle();
    double rawMag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
    // double cappedMag = Math.min(rawMag, 1);
    return new Translation2d(rawMag, direction);
  }

  Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
    Translation2d force = Translation2d.kZero;
    for (Obstacle obs : Field.OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Translation2d getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
  }

  public record RepulsorSample(Translation2d intermediateGoal, double vx, double vy) {}

  public RepulsorSample sampleField(
      Translation2d curTrans, double maxSpeed, double slowdownDistance) {
    Translation2d err = curTrans.minus(goal);
    Translation2d netForce = getForce(curTrans, goal);

    double stepSize_m;
    if (err.getNorm() < slowdownDistance) {
      stepSize_m =
          MathUtil.interpolate( // TODO: maybe don't divide by distance
              0, maxSpeed * Robot.defaultPeriodSecs, err.getNorm() / slowdownDistance);
    } else {
      stepSize_m = maxSpeed * Robot.defaultPeriodSecs;
    }
    Translation2d step = new Translation2d(stepSize_m, netForce.getAngle());
    return new RepulsorSample(
        curTrans.plus(step),
        step.getX() / Robot.defaultPeriodSecs,
        step.getY() / Robot.defaultPeriodSecs);
  }
}
