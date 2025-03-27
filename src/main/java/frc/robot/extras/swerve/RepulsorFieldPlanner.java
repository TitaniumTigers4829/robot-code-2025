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
import frc.robot.extras.math.forces.Force;
import frc.robot.extras.util.ReefLocations;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;

import org.ejml.equation.IntegerSequence.For;

public class RepulsorFieldPlanner {
  private abstract static class Obstacle {
    double strength;
    boolean positive;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist, double maxRange) {
      if (Math.abs(dist) > maxRange) {
        return 0;
      }
      if (MathUtil.isNear(0, dist, 1e-2)) {
        dist = 1e-2;
      }
      var forceMag = strength / (dist * dist);
      forceMag -= strength / (maxRange * maxRange);
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }
  }

  private static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailLength;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      Translation2d targetToLoc = loc.minus(target);
      Rotation2d targetToLocAngle = targetToLoc.getAngle();
      Translation2d sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);

      Translation2d positionToLocation = position.minus(loc);
      double positionToLocationDistance = positionToLocation.getNorm();
      Force outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Force(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Force.kZero;
      }

      Translation2d positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      double distanceAlongLine = positionToLine.getX();

      Force sidewaysForce;
      double distanceScalar = distanceAlongLine / tailLength;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        double secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        double distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / primaryMaxRange);
          } else {
            strength =
                -tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
                    + tailLength * tailStrength / (tailLength - primaryMaxRange);
          }
          strength *= 1 - distanceToLine / secondaryMaxRange;

          double sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          Rotation2d sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Force(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Force.kZero;
        }
      } else {
        sidewaysForce = Force.kZero;
      }

      return outwardsForce.plus(sidewaysForce);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    final double y;
    final double maxRange;

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > maxRange) {
        return Force.kZero;
      }
      return new Force(0, distToForceMag(y - position.getY(), maxRange));
    }
  }

  private static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > maxRange) {
        return Force.kZero;
      }
      return new Force(distToForceMag(x - position.getX(), maxRange), 0);
    }
  }

  private static class LineObstacle extends Obstacle {
    final Translation2d startPoint;
    final Translation2d endPoint;
    final double length;
    final Rotation2d angle;
    final Rotation2d inverseAngle;
    final double maxRange;

    public LineObstacle(Translation2d start, Translation2d end, double strength, double maxRange) {
      super(strength, true);
      startPoint = start;
      endPoint = end;
      var delta = end.minus(start);
      length = delta.getNorm();
      angle = delta.getAngle();
      inverseAngle = angle.unaryMinus();
      this.maxRange = maxRange;
    }

    @Override
    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var positionToLine = position.minus(startPoint).rotateBy(inverseAngle);
      if (positionToLine.getX() > 0 && positionToLine.getX() < length) {
        return new Force(
            Math.copySign(distToForceMag(positionToLine.getY(), maxRange), positionToLine.getY()),
            angle.rotateBy(Rotation2d.kCCW_90deg));
      }
      Translation2d closerPoint;
      if (positionToLine.getX() <= 0) {
        closerPoint = startPoint;
      } else {
        closerPoint = endPoint;
      }
      return new Force(
          distToForceMag(position.getDistance(closerPoint), maxRange),
          position.minus(closerPoint).getAngle());
    }
  }

  static final double FIELD_LENGTH = 17.6022;
  static final double FIELD_WIDTH = 8.1026;

  static final double SOURCE_X = 1.75;
  static final double SOURCE_Y = 1.25;
  static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          // Reef
          new TeardropObstacle(ReefLocations.BLUE_REEF, 1, 2, .83, 3, 2),
          new TeardropObstacle(ReefLocations.RED_REEF, 1, 2, .83, 3, 2),
          // Walls
          new HorizontalObstacle(0.0, 0.5, .5, true),
          new HorizontalObstacle(FIELD_WIDTH, 0.5, .5, false),
          new VerticalObstacle(0.0, 0.5, .5, true),
          new VerticalObstacle(FIELD_LENGTH, 0.5, .5, false),
          // Sources
          new LineObstacle(new Translation2d(0, SOURCE_Y), new Translation2d(SOURCE_X, 0), .5, .5),
          new LineObstacle(
              new Translation2d(0, FIELD_WIDTH - SOURCE_Y),
              new Translation2d(SOURCE_X, FIELD_WIDTH),
              .5,
              .5),
          new LineObstacle(
              new Translation2d(FIELD_LENGTH, SOURCE_Y),
              new Translation2d(FIELD_LENGTH - SOURCE_X, 0),
              .5,
              .5),
          new LineObstacle(
              new Translation2d(FIELD_LENGTH, FIELD_WIDTH - SOURCE_Y),
              new Translation2d(FIELD_LENGTH - SOURCE_X, FIELD_WIDTH),
              .5,
              .5));

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
            new Translation2d(x * FIELD_LENGTH / ARROWS_X, y * FIELD_WIDTH / ARROWS_Y);
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

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }
    Rotation2d direction = displacement.getAngle();
    double rawMag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
    // double cappedMag = Math.min(rawMag, 1);
    return new Force(rawMag, direction);
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    Force force = Force.kZero;
    for (Obstacle obs : FIELD_OBSTACLES) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
  }

  public record RepulsorSample(Translation2d intermediateGoal, double vx, double vy) {}

  public RepulsorSample sampleField(
      Translation2d curTrans, double maxSpeed, double slowdownDistance) {
    Translation2d err = curTrans.minus(goal);
    Force netForce = getForce(curTrans, goal);

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
