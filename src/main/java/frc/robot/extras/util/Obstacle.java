package frc.robot.extras.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.extras.math.forces.Force;

/**
 * Abstract base class for obstacles that exert a repulsive force.
 */
public abstract class Obstacle {
    protected double strength;
    protected boolean positive;

    /**
     * Constructs an Obstacle.
     *
     * @param strength the strength of the obstacle's repulsive effect
     * @param positive if true, the force is applied outward; if false, it is inverted
     */
    public Obstacle(double strength, boolean positive) {
        this.strength = strength;
        this.positive = positive;
    }

    /**
     * Computes the force exerted by this obstacle at the given position relative to the goal.
     *
     * @param position the position at which to compute the force
     * @param goal the goal or target position
     * @return a mutable vector representing the force
     */
    public abstract Force getForceAtPosition(Force position, Force goal);

    /**
     * Converts a distance to a force magnitude, based on the obstacle's strength and effective range.
     *
     * @param dist the distance from the obstacle
     * @param maxRange the maximum effective range of the obstacle
     * @return the calculated force magnitude
     */
    protected double distToForceMag(double dist, double maxRange) {
        if (MathUtil.isNear(0, dist, 1e-2)) {
            dist = 1e-2;
        }
        double forceMag = strength / (dist * dist);
        forceMag -= strength / (maxRange * maxRange);
        return positive ? forceMag : -forceMag;
    }

    /**
     * Helper method to rotate a value by a given angle (radians).
     *
     * @param radians the angle in radians
     * @param value the value to be rotated
     * @return the rotated value
     */
    protected double rotateBy(double radians, double value) {
        return Math.cos(radians) * value;
    }

    /**
     * A simple point obstacle that exerts force within a limited range.
     */
    public static class PointObstacle extends Obstacle {
        private final Force loc;
        private final double effectMaxRange = 0.5;
        private final Force positionToLoc = new Force();
        private final Force goalToPosition = new Force();
        private final Force outwardsVector = new Force();
        private final Force sidewaysVector = new Force();
        private final Force output = new Force();

        /**
         * Constructs a PointObstacle.
         *
         * @param loc the location of the obstacle
         * @param strength the repulsive strength of the obstacle
         * @param positive direction flag for the force
         */
        public PointObstacle(Force loc, double strength, boolean positive) {
            super(strength, positive);
            this.loc = loc;
        }

        @Override
        public Force getForceAtPosition(Force position, Force goal) {
            // Calculate the vector from position to obstacle location.
            positionToLoc.set(position);
            positionToLoc.minus(loc);

            // Calculate the vector from goal to position.
            goalToPosition.set(goal);
            goalToPosition.minus(position);

            // Reset output force.
            output.set(Translation2d.kZero);

            double dist = loc.getDistance(position);
            if (dist > effectMaxRange) {
                return output;
            }

            double outwardsMag = distToForceMag(dist, effectMaxRange);
            outwardsVector.setPolar(outwardsMag, positionToLoc.getAngle().getRadians());

            // Calculate an adjustment based on the angle between the goal-to-position and obstacle-to-position.
            Rotation2d theta = goalToPosition.getAngle().minus(position.minus(loc).getAngle());
            double magAdjustment = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

            sidewaysVector.set(outwardsVector);
            sidewaysVector.rotateBy(Rotation2d.kCCW_90deg);
            sidewaysVector.div(outwardsVector.getNorm());
            sidewaysVector.times(magAdjustment);

            output.set(outwardsVector);
            output.plus(sidewaysVector);
            return output;
        }
    }

    /**
     * A snowman obstacle that combines a primary and a secondary effect.
     */
    public static class SnowmanObstacle extends Obstacle {
        private final Force loc;
        private final double primaryMaxRange;
        private final double secondaryDistance;
        private final double secondaryMaxRange;
        private final double secondaryStrengthRatio;
        private final Force goalToLoc = new Force();
        private final Force sidewaysCircle = new Force();
        private final Force output = new Force();

        /**
         * Constructs a SnowmanObstacle.
         *
         * @param loc the location of the obstacle
         * @param primaryStrength the primary repulsive strength
         * @param primaryMaxRange the primary effective range
         * @param secondaryDistance the distance for the secondary effect
         * @param secondaryStrength the secondary repulsive strength
         * @param secondaryMaxRange the secondary effective range
         */
        public SnowmanObstacle(Force loc,
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
        public Force getForceAtPosition(Force position, Force goal) {
            output.set(Translation2d.kZero);
            goalToLoc.set(loc);
            goalToLoc.minus(goal);
            sidewaysCircle.setPolar(secondaryDistance, goalToLoc.getAngle().getRadians());
            sidewaysCircle.plus(loc);
            double dist = loc.getDistance(position);
            double sidewaysDist = sidewaysCircle.getDistance(position);
            if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
                return output;
            }
            double sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange)
                    / secondaryStrengthRatio;
            double outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
            Translation2d initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());
            Rotation2d sidewaysTheta = goal.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());
            double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
            goalToLoc.rotateBy(Rotation2d.kCCW_90deg);
            double sidewaysAngle = goalToLoc.getAngle().getRadians();
            output.setPolar(sideways, sidewaysAngle);
            output.plus(initial);
            return output;
        }
    }

    /**
     * A teardrop obstacle that has a primary circular effect with a tapered tail.
     */
    public static class TeardropObstacle extends Obstacle {
        private final Force loc;
        private final double primaryMaxRange;
        private final double primaryRadius;
        private final double tailStrength;
        private final double tailDistance;
        private final Force output = new Force();

        /**
         * Constructs a TeardropObstacle.
         *
         * @param loc the location of the obstacle
         * @param primaryStrength the primary repulsive strength
         * @param primaryMaxRange the primary effective range
         * @param primaryRadius the radius within which the primary effect is diminished
         * @param tailStrength the strength of the tail effect
         * @param tailLength the length of the tail (added to primaryMaxRange)
         */
        public TeardropObstacle(Force loc,
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
        public Force getForceAtPosition(Force position, Force goal) {
            Force targetToLoc = loc.minus(goal);
            Rotation2d targetToLocAngle = targetToLoc.getAngle();
            Force sidewaysPoint = new Force(tailDistance, targetToLoc.getAngle()).plus(loc);

            Force positionToLocation = position.minus(loc);
            double positionToLocationDistance = positionToLocation.getNorm();
            Force outwardsForce;
            if (positionToLocationDistance <= primaryMaxRange) {
                outwardsForce = new Force(
                        distToForceMag(Math.max(positionToLocationDistance - primaryRadius, 0),
                                       primaryMaxRange - primaryRadius),
                        positionToLocation.getAngle());
            } else {
                outwardsForce = Force.kZero;
            }

            Force positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
            double distanceAlongLine = positionToLine.getX();
            Force sidewaysForce;
            double distanceScalar = distanceAlongLine / tailDistance;
            if (distanceScalar >= 0 && distanceScalar <= 1) {
                double secondaryMaxRange = MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
                double distanceToLine = Math.abs(positionToLine.getY());
                if (distanceToLine <= secondaryMaxRange) {
                    double sidewaysMag = tailStrength * (1 - distanceScalar * distanceScalar)
                                          * (secondaryMaxRange - distanceToLine);
                    Rotation2d sidewaysTheta = goal.minus(position).getAngle()
                                              .minus(position.minus(sidewaysPoint).getAngle());
                    sidewaysForce = new Force(
                            sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                            targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
                } else {
                    sidewaysForce = Force.kZero;
                }
            } else {
                sidewaysForce = Force.kZero;
            }

            output.set(outwardsForce);
            output.plus(sidewaysForce);
            return output;
        }
    }

    /**
     * A horizontal obstacle that applies force based on a fixed Y-coordinate.
     */
    public static class HorizontalObstacle extends Obstacle {
        private final double y;
        private final double maxRange;
        private final Force output = new Force();

        /**
         * Constructs a HorizontalObstacle.
         *
         * @param y the Y-coordinate of the obstacle
         * @param strength the strength of the repulsive force
         * @param maxRange the effective range of the obstacle
         * @param positive direction flag for the force
         */
        public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
            super(strength, positive);
            this.y = y;
            this.maxRange = maxRange;
        }

        @Override
        public Force getForceAtPosition(Force position, Force goal) {
            output.set(Force.kZero);
            double dist = Math.abs(position.getY() - y);
            if (dist < maxRange) {
                output.set(0, distToForceMag(y - position.getY(), maxRange));
            }
            return output;
        }
    }

    /**
     * A vertical obstacle that applies force based on a fixed X-coordinate.
     */
    public static class VerticalObstacle extends Obstacle {
        private final double x;
        private final double maxRange;
        private final Force output = new Force();

        /**
         * Constructs a VerticalObstacle.
         *
         * @param x the X-coordinate of the obstacle
         * @param strength the strength of the repulsive force
         * @param maxRange the effective range of the obstacle
         * @param positive direction flag for the force
         */
        public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
            super(strength, positive);
            this.x = x;
            this.maxRange = maxRange;
        }

        @Override
        public Force getForceAtPosition(Force position, Force goal) {
            output.set(Force.kZero);
            double dist = Math.abs(position.getX() - x);
            if (dist < maxRange) {
                output.set(distToForceMag(x - position.getX(), maxRange), 0);
            }
            return output;
        }
    }
}
