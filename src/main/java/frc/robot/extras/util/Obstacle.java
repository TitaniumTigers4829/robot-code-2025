package frc.robot.extras.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class Obstacle {
  protected final double strength;
  protected final boolean positive;

  public Obstacle(double strength, boolean positive) {
    this.strength = strength;
    this.positive = positive;
  }

  /**
   * Computes the force exerted by this obstacle at the given position relative to the goal.
   *
   * @param position the position at which to compute the force
   * @param goal the goal or target position
   * @return an immutable vector representing the force
   */
  public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d goal);

  protected double distToForceMag(double dist, double maxRange) {
    if (MathUtil.isNear(0, dist, 1e-2)) {
      dist = 1e-2;
    }
    double forceMag = strength / (dist * dist);
    forceMag -= strength / (maxRange * maxRange);
    return positive ? forceMag : -forceMag;
  }

  // Helper method: rotates a value by a given angle (radians)
  protected double rotateBy(double radians, double value) {
    return Math.cos(radians) * value;
  }

  // Helper method for rotating an immutable Translation2d by a Rotation2d.
  private static Translation2d rotate(Translation2d vector, Rotation2d rotation) {
    double cos = Math.cos(rotation.getRadians());
    double sin = Math.sin(rotation.getRadians());
    return new Translation2d(
        vector.getX() * cos - vector.getY() * sin, vector.getX() * sin + vector.getY() * cos);
  }

  /** A simple point obstacle that exerts force within a limited range. */
  public static class PointObstacle extends Obstacle {
    private final Translation2d loc;
    private final double effectMaxRange = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      Translation2d positionToLoc = position.minus(loc);
      Translation2d goalToPosition = goal.minus(position);
      double dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return new Translation2d(0, 0);
      }
      double outwardsMag = distToForceMag(dist, effectMaxRange);
      double angle = positionToLoc.getAngle().getRadians();
      Translation2d outwardsVector =
          new Translation2d(outwardsMag * Math.cos(angle), outwardsMag * Math.sin(angle));

      // Calculate an adjustment based on the angle between the goal-to-position and
      // obstacle-to-position.
      Rotation2d theta = goalToPosition.getAngle().minus(positionToLoc.getAngle());
      double magAdjustment = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      // Create a sideways vector by rotating the outwards vector 90° CCW and normalizing.
      Translation2d sidewaysVector =
          new Translation2d(-outwardsVector.getY(), outwardsVector.getX());
      double norm = Math.hypot(sidewaysVector.getX(), sidewaysVector.getY());
      if (norm != 0) {
        sidewaysVector =
            new Translation2d(sidewaysVector.getX() / norm, sidewaysVector.getY() / norm);
      }
      sidewaysVector =
          new Translation2d(
              sidewaysVector.getX() * magAdjustment, sidewaysVector.getY() * magAdjustment);

      return new Translation2d(
          outwardsVector.getX() + sidewaysVector.getX(),
          outwardsVector.getY() + sidewaysVector.getY());
    }
  }

  /** A snowman obstacle that combines a primary and a secondary effect. */
  public static class SnowmanObstacle extends Obstacle {
    private final Translation2d loc;
    private final double primaryMaxRange;
    private final double secondaryDistance;
    private final double secondaryMaxRange;
    private final double secondaryStrengthRatio;

    public SnowmanObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double secondaryDistance,
        double secondaryStrength,
        double secondaryMaxRange) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.secondaryDistance = secondaryDistance;
      this.secondaryMaxRange = secondaryMaxRange;
      this.secondaryStrengthRatio = primaryStrength / secondaryStrength;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      // Compute vector from goal to obstacle
      Translation2d goalToLoc = loc.minus(goal);
      double angle = goalToLoc.getAngle().getRadians();
      // sidewaysCircle = loc + polar(secondaryDistance, angle)
      Translation2d sidewaysCircle =
          loc.plus(
              new Translation2d(
                  secondaryDistance * Math.cos(angle), secondaryDistance * Math.sin(angle)));
      double dist = loc.getDistance(position);
      double sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return new Translation2d(0, 0);
      }
      double sidewaysMag = distToForceMag(sidewaysDist, primaryMaxRange) / secondaryStrengthRatio;
      double outwardsMag = distToForceMag(dist, secondaryMaxRange);
      double initialAngle = position.minus(loc).getAngle().getRadians();
      Translation2d initial =
          new Translation2d(
              outwardsMag * Math.cos(initialAngle), outwardsMag * Math.sin(initialAngle));

      double angle1 = goal.minus(position).getAngle().getRadians();
      double angle2 = position.minus(sidewaysCircle).getAngle().getRadians();
      Rotation2d sidewaysTheta = new Rotation2d(angle1 - angle2);
      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));

      // Rotate goalToLoc 90° counter-clockwise
      Translation2d rotatedGoalToLoc = new Translation2d(-goalToLoc.getY(), goalToLoc.getX());
      double sidewaysAngle = rotatedGoalToLoc.getAngle().getRadians();
      Translation2d output =
          new Translation2d(sideways * Math.cos(sidewaysAngle), sideways * Math.sin(sidewaysAngle));
      return output.plus(initial);
    }
  }

  /** A teardrop obstacle that has a primary circular effect with a tapered tail. */
  public static class TeardropObstacle extends Obstacle {
    private final Translation2d loc;
    private final double primaryMaxRange;
    private final double primaryRadius;
    private final double tailStrength;
    private final double tailDistance;

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
      this.tailDistance = tailLength + primaryMaxRange;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      Translation2d targetToLoc = loc.minus(goal);
      Rotation2d targetToLocAngle = targetToLoc.getAngle();
      // sidewaysPoint = loc + polar(tailDistance, targetToLocAngle)
      Translation2d sidewaysPoint =
          loc.plus(
              new Translation2d(
                  tailDistance * Math.cos(targetToLocAngle.getRadians()),
                  tailDistance * Math.sin(targetToLocAngle.getRadians())));
      Translation2d positionToLocation = position.minus(loc);
      double positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        double forceMag =
            distToForceMag(
                Math.max(positionToLocationDistance - primaryRadius, 0),
                primaryMaxRange - primaryRadius);
        double angle = positionToLocation.getAngle().getRadians();
        outwardsForce = new Translation2d(forceMag * Math.cos(angle), forceMag * Math.sin(angle));
      } else {
        outwardsForce = new Translation2d(0, 0);
      }
      // Rotate (position - loc) by -targetToLocAngle.
      Translation2d positionToLine = rotate(position.minus(loc), targetToLocAngle.unaryMinus());
      double distanceAlongLine = positionToLine.getX();
      Translation2d sidewaysForce;
      double distanceScalar = distanceAlongLine / tailDistance;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        double secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        double distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double sidewaysMag =
              tailStrength
                  * (1 - distanceScalar * distanceScalar)
                  * (secondaryMaxRange - distanceToLine);
          double angle1 = goal.minus(position).getAngle().getRadians();
          double angle2 = position.minus(sidewaysPoint).getAngle().getRadians();
          Rotation2d sidewaysTheta = new Rotation2d(angle1 - angle2);
          double finalSidewaysMag = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
          Rotation2d rotated = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
          double rotAngle = rotated.getRadians();
          sidewaysForce =
              new Translation2d(
                  finalSidewaysMag * Math.cos(rotAngle), finalSidewaysMag * Math.sin(rotAngle));
        } else {
          sidewaysForce = new Translation2d(0, 0);
        }
      } else {
        sidewaysForce = new Translation2d(0, 0);
      }
      return outwardsForce.plus(sidewaysForce);
    }
  }

  /** A horizontal obstacle that applies force based on a fixed Y-coordinate. */
  public static class HorizontalObstacle extends Obstacle {
    private final double y;
    private final double maxRange;

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      double dist = Math.abs(position.getY() - y);
      if (dist < maxRange) {
        return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
      }
      return new Translation2d(0, 0);
    }
  }

  /** A vertical obstacle that applies force based on a fixed X-coordinate. */
  public static class VerticalObstacle extends Obstacle {
    private final double x;
    private final double maxRange;

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      double dist = Math.abs(position.getX() - x);
      if (dist < maxRange) {
        return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
      }
      return new Translation2d(0, 0);
    }
  }

  public static class LineObstacle extends Obstacle {
    private final Translation2d startPoint;
    private final Translation2d endPoint;
    private final double length;
    private final Rotation2d angle;
    private final Rotation2d inverseAngle;
    private final double maxRange;

    public LineObstacle(Translation2d start, Translation2d end, double strength, double maxRange) {
      super(strength, true);
      this.startPoint = start;
      this.endPoint = end;
      Translation2d delta = end.minus(start);
      this.length = delta.getNorm();
      this.angle = delta.getAngle();
      this.inverseAngle = angle.unaryMinus();
      this.maxRange = maxRange;
    }

    @Override
    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      // Compute the vector from the line's start to the current position.
      Translation2d relative = position.minus(startPoint);
      // Rotate the relative vector by the inverse angle to align with the line.
      Translation2d positionToLine = rotate(relative, inverseAngle);

      // If the projected point lies along the line segment, apply a force perpendicular to the
      // line.
      if (positionToLine.getX() > 0 && positionToLine.getX() < length) {
        double forceMag =
            Math.copySign(distToForceMag(positionToLine.getY(), maxRange), positionToLine.getY());
        Rotation2d rotatedAngle = angle.rotateBy(Rotation2d.kCCW_90deg);
        return new Translation2d(
            forceMag * Math.cos(rotatedAngle.getRadians()),
            forceMag * Math.sin(rotatedAngle.getRadians()));
      }

      // Otherwise, determine which endpoint of the line is closer to the position.
      Translation2d closerPoint = (positionToLine.getX() <= 0) ? startPoint : endPoint;
      double forceMag = distToForceMag(position.getDistance(closerPoint), maxRange);
      Rotation2d forceAngle = position.minus(closerPoint).getAngle();
      return new Translation2d(
          forceMag * Math.cos(forceAngle.getRadians()),
          forceMag * Math.sin(forceAngle.getRadians()));
    }
  }
}
