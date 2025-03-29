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

  // Helper: create a Translation2d from polar coordinates.
  protected Translation2d fromPolar(double magnitude, double angleRadians) {
    return new Translation2d(
        magnitude * Math.cos(angleRadians), magnitude * Math.sin(angleRadians));
  }

  // Helper: rotate a vector 90° counter-clockwise.
  protected Translation2d rotate90(Translation2d vector) {
    return new Translation2d(-vector.getY(), vector.getX());
  }

  // Helper method for rotating an immutable Translation2d by a Rotation2d.
  private static Translation2d rotate(Translation2d vector, Rotation2d rotation) {
    double cos = Math.cos(rotation.getRadians());
    double sin = Math.sin(rotation.getRadians());
    return new Translation2d(
        vector.getX() * cos - vector.getY() * sin, vector.getX() * sin + vector.getY() * cos);
  }

  // Helper for obstacles that are based on a center point.
  // Returns a radial force vector directed from center to position.
  protected Translation2d radialForce(
      Translation2d center, double maxRange, Translation2d position) {
    double dist = center.getDistance(position);
    if (dist > maxRange) {
      return new Translation2d(0, 0);
    }
    double mag = distToForceMag(dist, maxRange);
    double angle = position.minus(center).getAngle().getRadians();
    return fromPolar(mag, angle);
  }

  /**
   * Computes a lateral (sideways) force based on the difference between the goal direction and the
   * radial direction (from obstacle to position). The force is computed as: lateralForce = scale *
   * sign(sin(theta / divisor)) where theta is the angular difference between (goal - position) and
   * radialVector.
   *
   * @param radialVector the vector from the obstacle center (or secondary point) to the current
   *     position.
   * @param position the current position.
   * @param goal the goal position.
   * @param scale the scaling factor (for example, half of the radial force magnitude).
   * @param divisor divisor applied to theta before computing the sign (for a softer effect, try 2
   *     or 1).
   * @return a Translation2d representing the lateral force vector.
   */
  protected Translation2d lateralForce(
      Translation2d radialVector,
      Translation2d position,
      Translation2d goal,
      double scale,
      double divisor) {
    // Calculate the angular difference between the goal direction and the radial vector.
    Rotation2d theta = goal.minus(position).getAngle().minus(radialVector.getAngle());
    // Compute adjustment magnitude using the sine of (theta/divisor)
    double adjustment = scale * Math.signum(Math.sin(theta.getRadians() / divisor));
    // Determine the lateral direction by rotating the radial vector 90° counter-clockwise.
    Translation2d lateralDir = rotate90(radialVector);
    double norm = lateralDir.getNorm();
    if (norm != 0) {
      lateralDir = new Translation2d(lateralDir.getX() / norm, lateralDir.getY() / norm);
    }
    return fromPolar(adjustment, lateralDir.getAngle().getRadians());
  }

  // ----------------- Obstacle Implementations -----------------

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
      double dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return new Translation2d(0, 0);
      }
      // Compute radial (outwards) force.
      double outwardsMag = distToForceMag(dist, effectMaxRange);
      Translation2d outwardsVector = fromPolar(outwardsMag, positionToLoc.getAngle().getRadians());
      // Compute lateral force using the helper.
      Translation2d lateralVector = lateralForce(positionToLoc, position, goal, outwardsMag / 2, 2);
      return outwardsVector.plus(lateralVector);
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
      // Primary radial effect from the main center.
      Translation2d primaryForce = radialForce(loc, primaryMaxRange, position);

      // Secondary effect: compute an offset center (sidewaysCircle)
      double angle = loc.minus(goal).getAngle().getRadians();
      Translation2d sidewaysCircle = loc.plus(fromPolar(secondaryDistance, angle));
      double dist = loc.getDistance(position);
      double sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return new Translation2d(0, 0);
      }
      // Secondary radial force.
      double secondaryMag = distToForceMag(sidewaysDist, primaryMaxRange) / secondaryStrengthRatio;
      // Compute the lateral force relative to the secondary center.
      Translation2d lateralForceVec =
          lateralForce(position.minus(sidewaysCircle), position, goal, secondaryMag, 1);
      // Primary secondary effect from the offset center.
      double outwardsMag = distToForceMag(dist, secondaryMaxRange);
      double initialAngle = position.minus(loc).getAngle().getRadians();
      Translation2d initial = fromPolar(outwardsMag, initialAngle);

      return initial.plus(lateralForceVec).plus(primaryForce);
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
      Translation2d outwardsForce;
      Translation2d positionToLocation = position.minus(loc);
      double positionToLocationDistance = positionToLocation.getNorm();
      if (positionToLocationDistance <= primaryMaxRange) {
        double forceMag =
            distToForceMag(
                Math.max(positionToLocationDistance - primaryRadius, 0),
                primaryMaxRange - primaryRadius);
        double angle = positionToLocation.getAngle().getRadians();
        outwardsForce = fromPolar(forceMag, angle);
      } else {
        outwardsForce = new Translation2d(0, 0);
      }
      // Determine a secondary point along the tail.
      Translation2d targetToLoc = loc.minus(goal);
      Rotation2d targetToLocAngle = targetToLoc.getAngle();
      Translation2d sidewaysPoint =
          loc.plus(fromPolar(tailDistance, targetToLocAngle.getRadians()));
      // Compute distance along tail direction.
      Translation2d positionToLine = rotate(position.minus(loc), targetToLocAngle.unaryMinus());
      double distanceAlongLine = positionToLine.getX();
      Translation2d lateral = new Translation2d(0, 0);
      double distanceScalar = distanceAlongLine / tailDistance;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        double secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        double distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          // Use the lateralForce helper.
          double scale =
              tailStrength
                  * (1 - distanceScalar * distanceScalar)
                  * (secondaryMaxRange - distanceToLine);
          lateral = lateralForce(position.minus(sidewaysPoint), position, goal, scale, 1);
        }
      }
      return outwardsForce.plus(lateral);
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

  /** A line obstacle that computes force either along the line or from its endpoints. */
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
      Translation2d posLine = rotate(relative, inverseAngle);

      // If the projected point lies along the line segment, apply a force perpendicular to the
      // line.
      if (posLine.getX() > 0 && posLine.getX() < length) {
        double forceMag = Math.copySign(distToForceMag(posLine.getY(), maxRange), posLine.getY());
        Rotation2d perp = angle.rotateBy(Rotation2d.kCCW_90deg);
        return fromPolar(forceMag, perp.getRadians());
      }

      // Otherwise, determine which endpoint of the line is closer to the position.
      Translation2d closer = (posLine.getX() <= 0) ? startPoint : endPoint;
      double forceMag = distToForceMag(position.getDistance(closer), maxRange);
      double forceAngle = position.minus(closer).getAngle().getRadians();
      return fromPolar(forceMag, forceAngle);
    }
  }
}
